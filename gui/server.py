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

        ser.read_until(b"choice:")
        ser.write(b"0\n")
        ser.read_until(b"x,y:")
        ser.write(msg.encode())
        
        ser.read_until(b"success: ")
        if(ser.read(1) == b"1"):
            return {"success": True},200
        else:
            return {"success":False},200

    except Exception as e:
        app.logger.warning("ESP not connected")
        print(e)

        return {"success":False},200


@app.route("/grip",methods=["POST"])
def grip():
    try:
        msg = str(request.json["to_grip"]) + "\n"
        ser.read_until(b"choice:")
        ser.write(b"1\n")
        ser.read_until(b"release/grip:")
        ser.write(msg.encode())

    except Exception as e:
        app.logger.warning("ESP not connected")
        print(e)


    return "",200

@app.route("/rotate",methods=["POST"])
def rotate():
    try:
        msg = str(request.json["angle"]) + "\n"

        ser.read_until(b"choice:")
        ser.write(b"2\n")
        ser.read_until(b"base angle:")
        ser.write(msg.encode())

        ser.read_until(b"success: ")
        if (ser.read(1) == b"1"):
            return {"success":True},200
        else:
            return {"success":False},200

    except Exception as e:
        app.logger.warning("ESP not connected")
        print(e)
        return {"success":False},200






#app.run(host="0.0.0.0",port=5000)
app.run()

