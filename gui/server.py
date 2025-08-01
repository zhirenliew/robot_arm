from flask import Flask,render_template,request
import serial

app = Flask(__name__)
try:
    ser = serial.Serial("/dev/cu.usbserial-0001",115200,timeout=1)
except:
    app.logger.warning("Can't connect to ESP")


@app.route("/")
def index():
    return render_template("index.html")

# TODO how to deal with like overloading requests
# how to make it sequential instead of async
@app.route("/move",methods=["POST"])
def move():
    try:
        msg = str(request.json["x"]) + "," + str(request.json["y"]) + "\n"
        ser.write(msg.encode())
        ser.read_until(b"success: ")
        
        if(ser.read(1) == b"1"):
            return {"success": True},200
        else:
            return {"success":False},200

    except Exception as e:
        app.logger.warning("ESP not connected")

        return {"success":False},200

app.run()

