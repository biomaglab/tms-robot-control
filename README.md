# Robot_TMS
Robotized transcranial magnetic stimulation controlled by InVesalius Navigator (https://github.com/invesalius/invesalius3)


The first step is to run the relay_server.py script with an argument to set the server port

`python relay_server.py 5000`

Next, execute the main_loop.py script. Note: the main_loop should be run using the **Python Console**

`python main_loop.py`

After that, run the InVesalius app.py script (https://github.com/invesalius/invesalius3) with the --remote-host argument, specifying the same port as the relay server

 `python app.py --remote-host http://localhost:5000`

Please note that every time InVesalius is started, the main_loop is automatically restarted.
