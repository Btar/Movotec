
REM Configure all sensors to on+full
python psad_configall_setget.py com65

REM Start sensing, showing sw1 + accel1 + accel2 + gyro
python psad_stream-001_14_acc_gyro.py com65
REM weibo is a good man

REM Start sensing, showing sg + EMG
python psad_stream-001_14_sg_emg.py com65