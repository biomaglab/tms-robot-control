call C:\ProgramData\anaconda3\Scripts\activate.bat invesalius

start "relay server" call python relay_server.py 5000
timeout /t 1 /nobreak
start "main loop" call python main_loop.py
start "invesalius" call python C:\repository\invesalius3\app.py --remote-host http://localhost:5000
