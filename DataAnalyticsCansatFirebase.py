from firebase import firebase
import numpy as np
from datetime import datetime
# from sklearn import linear_model
# from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
import pandas as pd
import statistics  
#From Data

temperature =[]
humidity =[]
gasconcentration =[]
pressure =[]
altitude =[]
pm2_5 =[]
#From Data Statistics
sttemperature =[]
sthumidity =[]
stgasconcentration =[]
stpressure =[]
staltitude =[]
stpm2_5 =[]
#Time
Time = []
DateTime = datetime.now()
print(DateTime)
#DtaBase from firebase
Firebase = firebase.FirebaseApplication("https://cansat-biomech-default-rtdb.firebaseio.com")

init = 0
while init == 0:
    #Read instrucción to node red to go Data Analytics
    Instruction = Firebase.get('/Go', '')
    instructions = [] #Array to instruccion from firebase
    #Extracting valuees from array 
    for key, value in Instruction.items():
        # print(value)
        instructions.append(value)
        print(instructions)

    nummax_position = instructions[len(instructions)-1] #Final data from array
    print (nummax_position)

    if nummax_position == 1:
        #Cansat go sensing data
        #Time To sending Data
        Time.append(datetime.now())
        print(Time)
    elif nummax_position == 0:
        #Cansat sensed data
        init = 9999

print(Time)
# T = datetime.now()
# sec =T.strftime(':%S')
# print(T)
# print(sec)  

#Read realtime Database GET
#read = Firebase.get('/AirQuality', '')
Temperature = Firebase.get('/AirQuality/Temperature', '')
Humidity = Firebase.get('/AirQuality/Humidity', '')
GasConcentration = Firebase.get('/AirQuality/Gas', '')
Pressure = Firebase.get('/AirQuality/Pressure', '')
Altitude = Firebase.get('/AirQuality/Altitude', '')
Pm2_5 = Firebase.get('/AirQuality/Pm', '')
# print(Temperature)
# print(Humidity)
# print(GasConcentration)
# print(Pressure)
# print(Altitude)
# print(Pm2_5)

#Extract Data Values from Arrays
for key, value in Temperature.items():
    # print(value)
    temperature.append(value)
for key, value in Pressure.items():
    # print(value)
    pressure.append(value)
for key, value in Humidity.items():
    # print(value)
    humidity.append(value)
for key, value in Pm2_5.items():
    # print(value)
    pm2_5.append(value)
for key, value in GasConcentration.items():
    # print(value)
    gasconcentration.append(value)
for key, value in Altitude.items():
    # print(value)
    altitude.append(value)

sizearrays = [len(temperature),len(humidity),len(altitude),len(pressure),len(pm2_5),len(gasconcentration),len(Time)]
lenTemp = len(temperature)
lenHum = len(humidity)
lenAlt = len(altitude)
lenPres = len(pressure)
lenPm2_5 = len(pm2_5)
lenGas = len(gasconcentration)
lenTime = len(Time)

maxlength = 99999
for i in range (len(sizearrays)) : 
    if sizearrays[i] <= maxlength:
        maxlength = sizearrays[i]
print (maxlength)

#DATA size 
data = [temperature, humidity, altitude, pressure, pm2_5, gasconcentration]
sizeordenatedata = [temperature, humidity, altitude, pressure, pm2_5, gasconcentration, Time] #Include time
namekey = ["Temperature","Humidity","Altitude","Pressure","Pm2.5", "GasConcentration"]
for i in range (len(sizearrays)) :
    if sizearrays[i] > maxlength:
        del data[i] [maxlength:len(data[i])]
        # print (data[i])

#STATISTICS
Statistics = [sttemperature, sthumidity, staltitude, stpressure, stpm2_5, stgasconcentration]
for i in range (len(data)):
    s = pd.Series(data[i])
    Statistics [i] = s.describe()

print(Statistics)

# if Statistics[34] <= 50:
#     print(Statistics[34])
#     print("Air quality is Good")

# elif  51 <= Statistics[34] & Statistics[34] <= 100:
#     print(Statistics[34])
#     print("Air quality is Moderate")

# elif  101 <= Statistics[34] & Statistics[34] <= 150:
#     print(Statistics[34])
#     print("Air quality is Unhealthy for SensitiveGroups")

# elif  151 <= Statistics[34] & Statistics[34] <= 200:
#     print(Statistics[34])
#     print("Air quality is Unhealthy")
# elif  201 <= Statistics[34] & Statistics[34] <= 300:
#     print(Statistics[34])
#     print("Air quality is very Unhealthy")
# elif  301 <= Statistics[34] & Statistics[34] <= 500:
#     print(Statistics[34])
#     print("Air quality is Hardzardous")
# count    ######### [0]
# mean     ######### [1]
# std      ######### [2]
# min      #########  .
# 25%      #########  .
# 50%      #########  .
# 75%      #########  .
# max      #########  .

# plt.style.use('_mpl-gallery')

#Lineal Regresion to altitude (minimos cuadrados)

#Data Analysis Particle Material
fig, axs = plt.subplots(nrows=1, ncols=1)
axs.plot(Time,pm2_5,label ='pm2_5')
axs.set_title('Pm2.5')
axs.set_xlabel('time(Sec)')
axs.set_ylabel('Particles 2.5 microns(kg/m3)')
plt.legend()

fig, axs1 = plt.subplots(figsize = (16,8), nrows = 2,ncols = 3)
fig.suptitle('Comparative')
#Grafic Pressure Vs Altitude Vs Temperature
axs1[0,0].plot(pressure,color = 'k',label ='pressure')
axs1[0,0].plot(temperature,label = 'temperature')
axs1[0,0].plot(altitude,label = 'altitude')
axs1[0,0].plot(humidity,label = 'humidity')
axs1[0,0].plot(gasconcentration,label = 'gasconcentration')
axs1[0,0].plot(pm2_5,label = 'Pm2.5')
axs1[0,0].set_title('Tem, Press, Alt')
axs1[0,0].set_ylabel('value')
#Change pressure Respect Temperature:
axs1[0,1].set_title('pressure Respect Temperature')
axs1[0,1].hist2d(temperature,pressure)
# Change altitude Respect pressure:
axs1[0,2].set_title('altitude Respect pressure')
axs1[0,2].scatter(pressure, altitude, linestyle = '-', marker = '.')
#Linnear regresión
# X_train, X_test, Y_train, Y_test = train_test_split(pressure, altitude, test_size=0.2)
# lr = linear_model.LinearRegression()
# lr.fit(X_train, Y_train)
# Y_pred = lr.predict(X_test)
# axs1[0,2].plot(X_test, Y_pred, color = 'yellow', linewidth = 3)
# Change temperature Respect humidity:
axs1[1,0].set_title('temperature Respect humidity')
axs1[1,0].hist2d(humidity, temperature)
# Change humidity Respect pm2.5:
axs1[1,1].set_title('humidity Respect pm2.5')
axs1[1,1].hist2d(pm2_5, humidity)
# Change gasconcentration Respect pm2.5:
axs1[1,2].set_title('gasconcentration Respect pm2.5')
axs1[1,2].hist2d(pm2_5, gasconcentration)

fig, ax2 = plt.subplots(
    figsize = (6,10), nrows = 3, ncols = 2
)
fig.suptitle("Monitoring System anodeir quality")
current_ax = 0
for i in range (len(data)):
    ax2 =fig.axes[current_ax]
    ax2.plot(
            Time,
            data[i], 
            linewidth = .5
        )
    ax2.set_title(namekey[i])
    current_ax += 1
plt.show()

