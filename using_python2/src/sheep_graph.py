#!/usr/bin/env python
# license removed for brevity

import matplotlib.pyplot as plt
import numpy 

width = 0.3
x = [i for i in range(3)]
w = width/2
x1 = [i+w for i in range(3)]
x2 = [i-w for i in range(3)]
Tavg_split = [ 100.76509935931858 ,80.53540834184416,111.514190029196]

T_split_time_included = [ 100.76509935931858 + 15.671332454681396  ,80.53540834184416 + 18.921361351013 , 111.514190029196 + 21.581225442886353]

#Tavg_no_split = [62.809533526249986 , 149.1370267363158, 230.68267068263154]
T_avg_no_split_high_n = [111.69707337359998 , 101.67557608 , 157.34107617608694]

Std_split = [24.71996150179313, 16.03145689943332, 34.47367382671835]
#Std_no_split = [11.171387713488642, 18.117387204972943, 30.99905233938883]
Std_high_n = [11.171387713488642,   13.569025672707813 , 22.920537262178666]


#T3 = plt.bar(x1 , T_split_time_included , color = ((0.5, 0.0, 0.5)) , align = 'center' , width = width)
T1 = plt.bar(x1 , Tavg_split , color = ((1.0, 0.0, 0.0)) , yerr = Std_split ,align = 'center' ,width = width)
#T2 = plt.bar(x , Tavg_no_split , color = ((0.0, 0.0, 1.0)) , yerr = Std_no_split ,align = 'center' ,width = width)
T4 = plt.bar(x2 , T_avg_no_split_high_n , color = ((0.0, 1.0, 0.0)) , yerr = Std_high_n ,align = 'center' ,width = width)


plt.xticks(x , ('N=40' , 'N=100' , 'N=200' , 'N=500'), fontsize=15)
plt.legend((T4[0],T1[0]),('Without Splitting', ' With Splitting ') , loc = 2)
#plt.legend((T[0],T[1],T[2],T[3], T[4] , T[5]),(  ), loc =2 )

#plt.title('Avg Time for Completion with Equal Predators in Total', fontsize = 15)
plt.xlabel('Number of Sheep Agents' , fontsize = 20)
plt.ylabel('Average Time' , fontsize = 20)

plt.show()


############################### 500 AGENT COMPARISON
T_split = [716]
T_nosplit  = [1261]
split_time = [716 + 32.7361]
std_split = [230.6]
Std_no_split = [174.5]
width = 0.2
x = [0]
x1 = 0.2
x2 = x1 +  0.1 + width
#T3 = plt.bar(x2 , split_time , color = ((0.5, 0.0, 0.5)) , align = 'center' , width = width)
T1 = plt.bar(x2 , T_split , color = ((1.0, 0.0, 0.0)) ,  yerr = std_split ,align = 'center' , width = width)
T2 = plt.bar(x1 , T_nosplit , color = ((0.0, 1.0, 0.0)) , yerr = Std_no_split ,align = 'center' ,width = width)

plt.legend((T1[0] , T2[0]) , ('With Splitting' , 'Without Splitting ') , loc = 1)
#plt.title('Avg Time for 500 Agents',fontsize = 20)
plt.ylabel('Time(s)' , fontsize = 20)
plt.xlim(0,0.7)
plt.show()

############################### 40 sheep and varying predators comparison
width = 0.2
x = [0.2 , 0.5 , 0.8]
#T = [119.85474900541668 ,111.69707337359998, 116.3130071644 , 100.76509935931858]
T = [119.85474900541668 ,111.69707337359998, 116.3130071644]
#std = [15.724012915100246,14.764798318827163 ,13.867256372885217 , 24.71996150179313]
std = [15.724012915100246,14.764798318827163 ,13.867256372885217]

G = plt.bar(x , T , color = ((0.0,1.0,0.0),(0.0,1.0,0.0),(0.0,1.0,0.0)), yerr = std , align = 'center' , width = width )

plt.xlim(0,1.0)

plt.xticks(x, ('predators=2' , 'predators=4' , 'predators=6'), fontsize=15)
#plt.legend((G[0],G[1],G[2],G[3]) , ('No Splitting'  , 'Splitting'))
#plt.legend((G[0], G[3]) , ('Without Splitting'))


#plt.title('Varying the No. of Predators' ,fontsize = '20')
plt.ylabel('Time(s) to Complete' , fontsize = 20)
plt.xlabel('No. of Predators' , fontsize = 20)

plt.show()

################### same no. of predators per swarm , time comparison


# width = 0.3
# x = [i for i in range(3)]
# w = width/2
# x1 = [i+w for i in range(3)]
# x2 = [i-w for i in range(3)]

# Tavg_split = [  100.76509935931858  ,80.53540834184416,111.514190029196]

# T_split_time_included = [ 100.76509935931858 + 15.671332454681396  ,80.53540834184416 + 18.921361351013, 111.514190029196 + 21.581225442886353]

# #2 shepherds
# Tavg_no_split = [111.69707337359998 , 109.16561902541666 , 131.24869061538462]
# #T_avg_no_split_high_n = [62.809533526249986 , 101.67557608 , 152.44938511800004 , 1261.783634]

# Std_split = [ 24.71996150179313, 16.03145689943332, 34.47367382671835 ]
# Std_no_split = [14.764798318827163, 21.624970695693538 , 23.058959572863248]
# #Std_high_n = [11.171387713488642,   13.569025672707813 , 22.920537262178666]


# T3 = plt.bar(x1 , T_split_time_included , color = ((0.5, 0.0, 0.5)) , align = 'center' , width = width)
# T1 = plt.bar(x1 , Tavg_split , color = ((1.0, 0.0, 0.0)) , yerr = Std_split ,align = 'center' ,width = width)
# #T2 = plt.bar(x , Tavg_no_split , color = ((0.0, 0.0, 1.0)) , yerr = Std_no_split ,align = 'center' ,width = width)
# T4 = plt.bar(x2 , Tavg_no_split , color = ((0.0, 1.0, 0.0)) , yerr = Std_no_split ,align = 'center' ,width = width)


# plt.xticks(x , ('N=40' , 'N=100' , 'N=200'))
# plt.legend((T4[0],T1[0],T3[0]),('Without Splitting', ' With Splitting ' , 'Splitting Time') , loc = 2)
# #plt.legend((T[0],T[1],T[2],T[3], T[4] , T[5]),(  ), loc =2 )

# #plt.title('Avg Time for Completion with Equal Predators per Swarm', fontsize = 15)
# plt.xlabel('Number of Sheep Agents' , fontsize = 15)
# plt.ylabel('Average Time' , fontsize = 15)

# plt.show() 


