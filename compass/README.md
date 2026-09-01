# HOW TO RUN:

1. Create a virtual environment
```
python -m venv .venv
```

2. In the generated `pyvenv.cfg` file in the `.venv/` directory, change `include-system-site-packages` to `true`. It should look like this:
```
home = /usr/bin
include-system-site-packages = true
version = 3.13.5
executable = /usr/bin/python3.13
command = /usr/bin/python3 -m venv /home/<USER>/compass/.venv
```

3. Activate the virtual environment
```
source .venv/bin/activate
```

4. Install requirements with pip
```
pip install -r requirements.txt
```

5. Run the code using (make sure the virtual environment is still active)
```
python compass.py
```

The program will start displaying heading, pitch, and roll every 0.5 seconds.

NOTE: This program expects LEDs to be connected to GPIO pins 17 and 27, for displaying when the sensor is facing north and/or is level.
