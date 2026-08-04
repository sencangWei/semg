"""IMU 实时 3D 位姿 (位置 + 朝向) 可视化.

完全按参考图风格: 网格 + 紫色位置轨迹 + 当前位置的 RGB 姿态坐标系 + 坐标标签 + 位置数值.
位置由加速度航位推算 (dead reckoning, 有漂移, 后续 SLAM 修正); 静止时 ZUPT 冻结.

用法:
  1. 被 imu_client.py 调用 (加 --viz), 数据通过 multiprocessing.Queue 实时传入.
  2. 独立测试: python viz_imu_live.py --side left --axis-map +x,-y,-z

依赖:
  pip install pyqtgraph PyQt5 PyOpenGL numpy
"""

import argparse
import math
import sys
import time
from multiprocessing import Process, Queue as MPQueue

import numpy as np

from orientation_trajectory import (
    parse_axis_map,
    quaternion_conjugate,
    quaternion_multiply,
    normalize_quaternions,
    quaternion_to_euler_zyx,
)


def quat_to_matrix(q) -> np.ndarray:
    """四元数 [w,x,y,z] -> 3x3 旋转矩阵 (v_world = R @ v_sensor)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def matrix_to_axis_angle(R: np.ndarray) -> tuple:
    """3x3 旋转矩阵 -> (角度 deg, 单位轴)."""
    cos_a = (np.trace(R) - 1.0) / 2.0
    cos_a = max(-1.0, min(1.0, cos_a))
    angle = math.acos(cos_a)
    if angle < 1e-7:
        return 0.0, (0.0, 0.0, 1.0)
    s = 2.0 * math.sin(angle)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / s
    return math.degrees(angle), (float(axis[0]), float(axis[1]), float(axis[2]))


def quaternion_to_axis_angle(q: np.ndarray) -> tuple:
    """四元数 [w,x,y,z] 转 (角度 deg, 轴)."""
    w, x, y, z = q
    w = max(-1.0, min(1.0, w))
    angle = 2.0 * math.acos(w)
    s = math.sqrt(1.0 - w * w)
    if s < 1e-8:
        return 0.0, (0.0, 0.0, 1.0)
    return math.degrees(angle), (x / s, y / s, z / s)


def _viz_process_main(queue: MPQueue, side: str, axis_map: str,
                      baseline_s: float, title: str):
    """在独立进程中运行的 Qt/pyqtgraph 可视化主循环."""
    try:
        import pyqtgraph as pg
        import pyqtgraph.opengl as gl
        from PyQt5 import QtWidgets, QtCore, QtGui
    except Exception as e:
        print(f"[VIZ] 启动失败,缺少依赖: {e}")
        print("[VIZ] 请运行: pip install pyqtgraph PyQt5 PyOpenGL")
        return

    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)

    view = gl.GLViewWidget()
    view.setBackgroundColor((24, 26, 31))   # 深色底: GL additive/translucent 都可见 (白底会隐形)
    view.setWindowTitle(title)
    # 3D 透视视角 (与参考图一致), +Z 朝上
    view.setCameraPosition(distance=2.5, elevation=25, azimuth=45)

    # 左右手窗口自动并排, 避免重叠
    try:
        screen = app.primaryScreen().availableGeometry()
        win_w = screen.width() // 2 - 12
        win_h = min(640, screen.height() - 100)
        x = screen.left() + (6 if side == "left" else screen.width() // 2 + 6)
        view.setGeometry(int(x), int(screen.top() + 40), int(win_w), int(win_h))
    except Exception as e:
        print(f"[VIZ] 窗口定位失败(忽略): {e}")
    view.show()

    # 轴说明浮层 (QLabel, Qt 原生一定显示; GLTextItem 在很多环境不渲染)
    legend = QtWidgets.QLabel(
        "<span style='color:#ff6b6b'>■ X 四指指向</span> &nbsp; "
        "<span style='color:#66e08a'>■ Y 手心向内</span> &nbsp; "
        "<span style='color:#8aaaff'>■ Z 拇指(上)</span> &nbsp; 网格 0.1m/格"
    )
    legend.setStyleSheet("background:rgba(0,0,0,150); padding:6px; border-radius:4px;")
    legend.setParent(view)
    legend.move(8, 8)
    legend.setTextFormat(QtCore.Qt.RichText)
    legend.show()
    pos_label = QtWidgets.QLabel("--")
    pos_label.setStyleSheet("color:#dddddd; background:rgba(0,0,0,150); padding:6px; "
                            "border-radius:4px; font-family:monospace;")
    pos_label.setParent(view)
    pos_label.move(8, view.height() - 40)
    pos_label.show()

    # ---- 静态场景: 正方体内角三个相邻面 (XY底/XZ墙/YZ墙, 正卦限[0,L], 汇于原点) + 米刻度 ----
    GEXT = 0.5      # 网格范围 (米)
    GSTEP = 0.1     # 网格间距 (米)

    def _grid_face(va, vb, fix_idx, fix_val, color=(0.5, 0.5, 0.55, 0.55)):
        segs = []
        vals = np.arange(0.0, GEXT + GSTEP * 0.001, GSTEP)
        for v in vals:
            p1 = [0.0, 0.0, 0.0]; p2 = [0.0, 0.0, 0.0]
            p1[fix_idx] = fix_val; p2[fix_idx] = fix_val
            p1[vb] = v; p2[vb] = v; p1[va] = 0.0; p2[va] = GEXT
            segs += [p1, p2]
            q1 = [0.0, 0.0, 0.0]; q2 = [0.0, 0.0, 0.0]
            q1[fix_idx] = fix_val; q2[fix_idx] = fix_val
            q1[va] = v; q2[va] = v; q1[vb] = 0.0; q2[vb] = GEXT
            segs += [q1, q2]
        return gl.GLLinePlotItem(pos=np.array(segs, dtype=float), color=color,
                                 width=1.0, mode="lines", antialias=True,
                                 glOptions="translucent")
    static_items = []   # 网格+坐标轴, baseline 后统一转进对齐系
    for face in [_grid_face(0, 1, 2, 0.0), _grid_face(0, 2, 1, 0.0), _grid_face(1, 2, 0, 0.0)]:
        static_items.append(face)
        view.addItem(face)

    # 米刻度 (沿三条棱, 每 0.1m 一个)
    font_tick = QtGui.QFont("Arial", 8)
    try:
        for v in np.arange(0.0, GEXT + GSTEP * 0.001, GSTEP):
            view.addItem(gl.GLTextItem(pos=(v, 0.0, 0.0), color=(0.7, 0.5, 0.5, 1.0),
                                       text=f"{v:.1f}", font=font_tick))   # X 棱
            view.addItem(gl.GLTextItem(pos=(0.0, v, 0.0), color=(0.5, 0.75, 0.55, 1.0),
                                       text=f"{v:.1f}", font=font_tick))   # Y 棱
            view.addItem(gl.GLTextItem(pos=(0.0, 0.0, v), color=(0.55, 0.65, 0.95, 1.0),
                                       text=f"{v:.1f}", font=font_tick))   # Z 棱
    except Exception as e:
        print(f"[VIZ] 米刻度标注失败(忽略): {e}")

    WL = 0.5  # 世界坐标轴长度 (米, 与网格棱齐)
    world_triad = []
    for vec, col in [((WL, 0, 0), (0.95, 0.30, 0.30, 1.0)),
                     ((0, WL, 0), (0.30, 0.90, 0.40, 1.0)),
                     ((0, 0, WL), (0.35, 0.55, 1.0, 1.0))]:
        it = gl.GLLinePlotItem(pos=np.array([[0.0, 0.0, 0.0], vec], dtype=float),
                               color=col, width=4.0, antialias=True,
                               glOptions="translucent")
        view.addItem(it)
        world_triad.append(it)
        static_items.append(it)   # 坐标轴也转进对齐系

    font_big = QtGui.QFont("Arial", 13)
    font_big.setBold(True)
    try:
        view.addItem(gl.GLTextItem(pos=(WL + 0.05, 0.0, 0.0), color=(1.0, 0.5, 0.5, 1.0),
                                   text="X 四指指向 (m)", font=font_big))
        view.addItem(gl.GLTextItem(pos=(0.0, WL + 0.05, 0.0), color=(0.5, 0.95, 0.55, 1.0),
                                   text="Y 手心向内 (m)", font=font_big))
        view.addItem(gl.GLTextItem(pos=(0.0, 0.0, WL + 0.05), color=(0.55, 0.7, 1.0, 1.0),
                                   text="Z 拇指方向 (m)", font=font_big))
    except Exception as e:
        print(f"[VIZ] 坐标标注失败(忽略): {e}")

    # ---- 紫色位置轨迹 (中立对齐显示) + 当前位置标记 ----
    # 轴镜像 [X四指, Y手心, Z拇指]: 方向反了就把对应位置改成 -1.0
    MIRROR = np.array([1.0, -1.0, 1.0])   # 默认翻 Y (手心/左右)
    trail_item = gl.GLLinePlotItem(
        pos=np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
        color=(0.65, 0.25, 0.95, 1.0), width=2.8, antialias=True,
        glOptions="translucent")
    view.addItem(trail_item)
    tip_item = gl.GLScatterPlotItem(
        pos=np.array([[0.0, 0.0, 0.0]]),
        color=(0.85, 0.45, 1.0, 1.0), size=12.0,
        glOptions="translucent")
    view.addItem(tip_item)

    # ---- 当前位姿坐标系: RGB 三轴, 跟随位置 + 朝向 ----
    # 按手修正显示方向 (实测): 左手拇指反, 右手手心+拇指反. [红四指, 绿手心, 蓝拇指]
    ARROW = 0.14
    frame_signs = {"left":  (1.0,  1.0, -1.0),   # 左手: 翻蓝(拇指)
                   "right": (1.0, -1.0, -1.0)}   # 右手: 翻绿(手心)+蓝(拇指)
    signs = frame_signs.get(side, (1.0, 1.0, 1.0))
    frame_items = []
    for (vec, col, sg) in [((ARROW, 0, 0), (1.0, 0.20, 0.20, 1.0), signs[0]),
                           ((0, ARROW, 0), (0.30, 1.0, 0.35, 1.0), signs[1]),
                           ((0, 0, ARROW), (0.30, 0.45, 1.0, 1.0), signs[2])]:
        v = np.array(vec, dtype=float) * sg
        it = gl.GLLinePlotItem(pos=np.array([[0.0, 0.0, 0.0], v], dtype=float),
                               color=col, width=3.5, glOptions="translucent")
        view.addItem(it)
        frame_items.append(it)

    # ---- 坐标变换: axis_map (传感器->解剖) + 数据->可视化 ----
    try:
        basis = parse_axis_map(axis_map)
    except Exception as e:
        print(f"[VIZ] axis-map 解析失败: {e},使用默认值 +x,+y,+z")
        basis = parse_axis_map("+x,+y,+z")
    # basis: 传感器系 -> 解剖系 (X四指/Y手心/Z拇指), 用于姿态坐标系与欧拉读数

    # ---- 状态 ----
    POS_SCALE = 1.5          # 位置显示放大倍数 (网格已是真实米, 标题读数为真实米)
    STATIC_ACCEL = 0.25      # m/s^2, 线性加速度幅值低于此认为静止 (加速度ZUPT, 冻结不漂)
    pos = np.zeros(3)
    vel = np.zeros(3)
    g_world = None           # 世界系重力常量 (静止时估一次, 无滞后)
    lacc_bias = np.zeros(3)  # 世界系线性加速度偏置 (静止时估一次, 减掉防单向漂移)
    R_align = np.eye(3, dtype=np.float64)   # 中立姿态对齐旋转: 把四指方向对齐到世界+X
    prev_ts = None
    trail = []

    baseline_buffer = []
    baseline_q = None
    latest = None
    last_frame_wall = None
    got_first_frame = False

    def _close():
        try:
            view.close()
        except Exception:
            pass
        app.quit()

    def update():
        nonlocal latest, baseline_q, last_frame_wall, got_first_frame
        nonlocal pos, vel, g_world, lacc_bias, R_align, prev_ts

        # 取最新帧
        stop_flag = False
        while not queue.empty():
            try:
                msg = queue.get_nowait()
            except Exception:
                break
            if isinstance(msg, dict) and msg.get("stop"):
                stop_flag = True
                continue
            latest = msg
            last_frame_wall = time.time()
            got_first_frame = True
            if baseline_q is None and latest is not None:
                baseline_buffer.append(latest)

        if stop_flag:
            _close()
            return
        # 看门狗: 收到过帧后 2s 无新帧 -> 自动关窗
        if got_first_frame and last_frame_wall is not None:
            if time.time() - last_frame_wall > 2.0:
                _close()
                return

        # 捕获基准姿态
        if baseline_q is None and baseline_buffer:
            ts0 = baseline_buffer[0].get("wall_ts", 0)
            ts1 = baseline_buffer[-1].get("wall_ts", 0)
            if ts1 - ts0 >= baseline_s:
                qs = np.array([f["q6"] for f in baseline_buffer], dtype=np.float64)
                q_ref, _ = normalize_quaternions(np.mean(qs, axis=0, keepdims=True))
                baseline_q = q_ref[0]
                # 用静止段估计世界系重力常量 (含偏置); 之后减固定值 -> 无滞后, 不回弹
                aws = []
                for f in baseline_buffer:
                    if f.get("accel") is None:
                        continue
                    qq = np.asarray(f["q6"], dtype=np.float64)
                    qq = qq / (np.linalg.norm(qq) + 1e-8)
                    aws.append(quat_to_matrix(qq) @ np.asarray(f["accel"], dtype=np.float64))
                g_world = np.mean(np.array(aws), axis=0) if aws else np.zeros(3)
                # 估计世界系线性加速度偏置 (静止段平均), 减掉防单向漂移 (左移却往右走)
                laws = []
                for f in baseline_buffer:
                    la = f.get("lacc")
                    if la is None:
                        continue
                    qq = np.asarray(f["q6"], dtype=np.float64)
                    qq = qq / (np.linalg.norm(qq) + 1e-8)
                    laws.append(quat_to_matrix(qq) @ np.asarray(la, dtype=np.float64))
                lacc_bias = np.mean(np.array(laws), axis=0) if laws else np.zeros(3)
                # ---- 中立姿态对齐: 把"四指方向(解剖 +X)"对齐到基坐标 +X ----
                # 四指在世界系 = R(baseline_q) @ basis^{-1} @ [1,0,0]; 取水平偏航, 反向旋转
                R_base = quat_to_matrix(baseline_q)
                fingers_world = R_base @ (basis.T @ np.array([1.0, 0.0, 0.0]))
                yaw = math.atan2(fingers_world[1], fingers_world[0])
                c, s = math.cos(yaw), math.sin(yaw)
                R_align = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
                # 把基坐标系(网格+世界轴)也转进对齐系, 两只手朝向一致
                M4 = np.eye(4, dtype=np.float64)
                M4[:3, :3] = R_align
                tr_align = pg.Transform3D(M4)
                for it in static_items:
                    try:
                        it.setTransform(tr_align)
                    except Exception:
                        pass
                # 基坐标系已对齐, 摄像机用固定角度 (两只手一致): 沿基 +X(四指)看, 进屏幕
                view.setCameraPosition(azimuth=180.0, elevation=18)
                pos[:] = 0.0
                vel[:] = 0.0
                prev_ts = None
                trail.clear()
                print(f"[VIZ] {side} 基准姿态已捕获({len(baseline_buffer)} 帧), "
                      f"基坐标系对齐偏航={math.degrees(yaw):.1f}°, 开始渲染")

        if latest is None or baseline_q is None:
            return

        q6 = np.asarray(latest["q6"], dtype=np.float64)
        q6 = q6 / (np.linalg.norm(q6) + 1e-8)
        ts = latest.get("wall_ts")
        accel = latest.get("accel")
        gyro = latest.get("gyro")
        lacc = latest.get("lacc")   # 硬件线性加速度 (BNO085 已去重力+去偏置)

        # ---- 位置航位推算: 纯积分 + 缓慢漏桶 (不用陀螺仪 ZUPT, 否则纯平移会卡死) ----
        # 优先用硬件线性加速度 (最干净); 旧固件没有则回退: 加速度减固定重力
        if prev_ts is not None and ts is not None:
            dt = ts - prev_ts
            if 0.0 < dt < 0.2:
                if lacc is not None:
                    a_lin = quat_to_matrix(q6) @ np.asarray(lacc, dtype=np.float64) - lacc_bias
                elif accel is not None and g_world is not None:
                    a_lin = quat_to_matrix(q6) @ np.asarray(accel, dtype=np.float64) - g_world
                else:
                    a_lin = None
                if a_lin is not None:
                    # 加速度静止检测: |a| 小 -> 真静止 -> 冻结 (不漂); |a| 大 -> 在动 -> 积分
                    if float(np.linalg.norm(a_lin)) < STATIC_ACCEL:
                        vel[:] = 0.0
                        # 在线偏置更新: 静止时把残差慢慢并入偏置, 消除持续下漂(重力没去干净)
                        if lacc is not None:
                            lacc_bias = lacc_bias + a_lin * (dt / 2.0)
                    else:
                        a_lin[np.abs(a_lin) < 0.08] = 0.0      # 死区
                        vel += a_lin * dt
                        vel *= math.exp(-dt / 5.0)             # 缓慢漏桶, 抑制漂移
                        pos += vel * dt
                        if np.linalg.norm(pos) > 1.5:
                            pos[:] = 0.0
                            vel[:] = 0.0
                            trail.clear()
                        else:
                            trail.append(pos.copy() * POS_SCALE)
                            if len(trail) > 3000:
                                trail.pop(0)
        if ts is not None:
            prev_ts = ts

        # ---- 朝向 ----
        # 数据系欧拉角 (roll_x/pitch_y/yaw_z, 仅用于读数显示)
        q_rel_data = quaternion_multiply(
            quaternion_conjugate(baseline_q[np.newaxis, :]),
            q6[np.newaxis, :],
        )[0]
        q_rel_data[1:] = q_rel_data[1:] @ basis.T
        euler_deg = np.degrees(quaternion_to_euler_zyx(q_rel_data[np.newaxis, :])[0])

        # 姿态坐标系: 绝对解剖朝向 (红=四指实际方向, 绿=手心, 蓝=拇指), 对齐世界系
        # R_frame 的列 = R_align @ R(q6) @ basis^{-1} @ 单位轴 => 三色箭头跟随手的真实朝向
        R_frame = R_align @ quat_to_matrix(q6) @ basis.T
        angle, axis = matrix_to_axis_angle(R_frame)

        # 显示坐标: 先用 R_align 把轨迹转到"中立对齐系"(X=四指/Y=手心/Z=拇指), 再按轴镜像
        aligned = R_align @ pos
        disp = aligned * POS_SCALE * MIRROR
        tr = pg.Transform3D()
        tr.translate(float(disp[0]), float(disp[1]), float(disp[2]))
        if abs(angle) > 1e-3:
            tr.rotate(angle, *axis)
        for it in frame_items:
            it.setTransform(tr)

        # 更新轨迹 (对齐 + 镜像) 与当前位置标记
        if len(trail) >= 2:
            pts = np.asarray(trail, dtype=np.float64) @ R_align.T * MIRROR
            trail_item.setData(pos=pts)
        tip_item.setData(pos=np.array([disp], dtype=np.float64))

        # 标题 + 左下角浮层: 对齐系位置 (m, X四指/Y手心/Z拇指) + 朝向 (deg)
        pos_txt = (f"位置 X四指={aligned[0]:+.2f}m  Y手心={aligned[1]:+.2f}m  Z拇指={aligned[2]:+.2f}m\n"
                   f"朝向 R={euler_deg[0]:+.0f}° P={euler_deg[1]:+.0f}° Yaw={euler_deg[2]:+.0f}°")
        try:
            pos_label.setText(pos_txt)
            pos_label.adjustSize()
        except Exception:
            pass
        view.setWindowTitle(f"{title}  |  {pos_txt.replace(chr(10), '  ')}")

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(33)  # ~30 fps

    print(f"[VIZ] {side} 可视化窗口已启动,请保持 IMU 静止 {baseline_s}s 以捕获基准姿态")
    app.exec_()


def start_viz_process(side: str, axis_map: str = "+x,+y,+z",
                      baseline_s: float = 2.0) -> tuple[MPQueue, Process]:
    """启动可视化子进程,返回 (queue, process)."""
    q = MPQueue(maxsize=500)
    title = f"IMU Live - {side.upper()}"
    p = Process(
        target=_viz_process_main,
        args=(q, side, axis_map, baseline_s, title),
        daemon=False,
    )
    p.start()
    return q, p


def main():
    ap = argparse.ArgumentParser(description="IMU 实时 3D 位姿可视化")
    ap.add_argument("--side", default="left", help="left 或 right")
    ap.add_argument("--axis-map", default="+x,+y,+z",
                    help="轴映射,例如 +x,-y,-z")
    ap.add_argument("--baseline-s", type=float, default=2.0,
                    help="捕获基准姿态的秒数")
    args = ap.parse_args()

    q, p = start_viz_process(args.side, args.axis_map, args.baseline_s)
    print("可视化进程运行中,按 Ctrl+C 退出")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        p.terminate()
        p.join(timeout=2)


if __name__ == "__main__":
    main()
