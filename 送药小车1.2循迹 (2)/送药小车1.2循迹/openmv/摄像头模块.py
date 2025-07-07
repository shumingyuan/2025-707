import sensor, image, time,pyb
import display

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_vflip(True)
sensor.set_hmirror(True)
uart = pyb.UART(3, 9600)
lcd = display.SPIDisplay()
THRESHOLD = (18, 55, 27, 127, -17, 127)
clock = time.clock()
while(True):
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD])
    line = img.get_regression([(100,100)], robust = True)
    if (line):
        rho_err = abs(line.rho())-img.width()/2
        if line.theta()>90:
            theta_err = line.theta()-180
        else:
            theta_err = line.theta()
        img.draw_line(line.line(), color = 127)
        lcd.write(img,x_scale=-0.5,y_scale=-0.5)
        print(rho_err,line.magnitude(),rho_err)
        uart.write("@%.1f,%.1f,%d\r\n" % (
            rho_err,
            theta_err,
            line.magnitude()
        ))
        print("偏差:%.1fpx 角度:%.1f° 置信度:%d" %
             (rho_err, theta_err, line.magnitude()))
    else:
        lcd.write(img,x=160,y=90,x_scale=0.5,y_scale=0.5)
        uart.write("@null\r\n")
        print("未检测到黑线")
