# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt

Book1 = np.loadtxt('/home/idriver/Desktop/Book1.txt')
Book2 = np.loadtxt('/home/idriver/Desktop/Book2.txt')

x1=Book1[:,2];

data_verification=Book1[:,0];
total=Book1[:,1];

z_data_verification = np.polyfit(x1, data_verification, 3)
z_total = np.polyfit(x1, total, 3)

y_data_verification=np.polyval(z_data_verification,x1)
y_total=np.polyval(z_total,x1)

key_point1 = np.polyval(z_total,1)
key_point2 = np.polyval(z_total,5)
key_point3 = np.polyval(z_total,10)

plt.figure('Data Size vs Run Time')
p4, =plt.plot( x1, y_data_verification, 'k*-')
p5, =plt.plot( x1, y_total, 'g*-')

plt.plot(1, key_point1, 'ko')
plt.plot(5, key_point2, 'ko')
plt.plot(10, key_point3, 'ko')
plt.text(1, key_point1 + 1000, round(key_point1,2), {'color': 'k', 'ha': 'center', 'fontsize': 10})
plt.text(5, key_point2 + 1000, round(key_point2,2), {'color': 'k', 'ha': 'center', 'fontsize': 10})
plt.text(10, key_point3 + 1500, round(key_point3,2), {'color': 'k', 'ha': 'center', 'fontsize': 10})

plt.title("Data Size vs Run Time", fontsize=16)
plt.grid()
plt.legend([p4,p5],["data_verification","total"])
plt.xlabel('Data Size (GB)')
plt.ylabel('Run Time (s)')

x2=Book2[:,2];

data_verification2=Book2[:,0];
total2=Book2[:,1];

z_data_verification2 = np.polyfit(x2, data_verification2, 3)
z_total2 = np.polyfit(x2, total2, 3)

y_data_verification2=np.polyval(z_data_verification2,x2)
y_total2=np.polyval(z_total2,x2)

key_point11 = np.polyval(z_total2,5000)
key_point12 = np.polyval(z_total2,10000)
key_point13 = np.polyval(z_total2,40000)

plt.figure('Aera vs Run Time')
p14, =plt.plot( x2, y_data_verification2, 'k*-')
p15, =plt.plot( x2, y_total2, 'g*-')

plt.plot(5000, key_point11, 'ko')
plt.plot(10000, key_point12, 'ko')
plt.plot(40000, key_point13, 'ko')
plt.text(5000, key_point11 + 1200, round(key_point11,2), {'color': 'k', 'ha': 'center', 'fontsize': 10})
plt.text(10000, key_point12 + 1500, round(key_point12,2), {'color': 'k', 'ha': 'center', 'fontsize': 10})
plt.text(40000, key_point13 + 1500, round(key_point13,2), {'color': 'k', 'ha': 'center', 'fontsize': 10})

plt.title("Aera vs Run Time", fontsize=16)
plt.grid()
plt.legend([p14,p15],["data_verification","total"])
plt.xlabel('Aera(m2)')
plt.ylabel('Run Time (s)')


plt.show()