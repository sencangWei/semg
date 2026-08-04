@echo off
REM 安装 IMU 实时可视化依赖
REM 在采集用的 cmd/PowerShell 里运行: install_viz_deps.bat
python -m pip install pyqtgraph PyQt5 PyOpenGL PyOpenGL-accelerate
python -c "import pyqtgraph.opengl as gl; import PyQt5; print('viz deps OK')"
pause
