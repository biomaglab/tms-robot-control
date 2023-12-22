set SITE=sitename
set ROBOT=robotname
set USE_FORCE_SENSOR=false

start "relay server" call python relay_server.py 5000

timeout /t 1 /nobreak

start "main loop" call python main_loop.py
