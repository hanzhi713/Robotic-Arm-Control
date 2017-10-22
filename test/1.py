from Protocol import ServoProtocol
sp = ServoProtocol("COM3", 19200)
sp.write([90, 90, 90, 90, 90, 45])
sp.close()