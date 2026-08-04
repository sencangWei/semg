# 安装 IMU 实时可视化依赖
# 在采集用的 PowerShell/终端里运行: .\install_viz_deps.ps1
python -m pip install pyqtgraph PyQt5 PyOpenGL PyOpenGL-accelerate
python -c "import pyqtgraph.opengl as gl; import PyQt5; print('viz deps OK')"
Write-Host "按任意键继续..."
$Host.UI.RawUI.ReadKey("NoEcho,IncludeKeyDown") | Out-Null
