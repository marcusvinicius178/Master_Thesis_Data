#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 28 14:51:37 2021

@author: autoware-auto-ros1
"""

from Power_Consumption_Astar_Mechanic_Energy import *
from Power_Consumption_Lattice_Mechanic_Energy import *
from Power_Consumption_Public_Road_Mechanic_Energy import *
from Power_Consumption_OP_Mechanic_Energy import *
import matplotlib.pyplot as plt
import seaborn as sns

#Comment the line below and one of the block below (Acceleration on Brake Power) to plot individual figures
#seaborn cannot plot different figures as matplolib, just share the same figure in subplots...
f,axes = plt.subplots(2,1)


Op_consumption_hp = Mean_Op_Acceleration_power
Astar_Consumption_hp = Mean_Astar_Acceleration_power
Lattice_Consumption_hp = Median_Lattice_Acceleration_power
Public_Road_consumption = Mean_Public_Road_Acceleration_power

Data_Dictionary = {'A*':Astar_Consumption_hp, 'Lattice':Lattice_Consumption_hp, 'Op':Op_consumption_hp, 'Public_Road':Public_Road_consumption  }
data_items = Data_Dictionary.items()
data_List = list(data_items)
Planners_Consumption_Df = pd.DataFrame(data_List)
Planners_Consumption_Df.columns = ["Trajectory Planners", "Power Consumption(Hp)"]

Planners_Consumption_Df.to_csv('df_csv', index=False)


sns.set_theme(style="whitegrid")
#Plot Vertically
#ax = sns.barplot(x= "Trajectory Planners", y="Power Consumption(Hp)" , data = Planners_Consumption_Df)
#Plot Horizontally
#ax = sns.barplot(x= "Power Consumption(Hp)", y= "Trajectory Planners" , data = Planners_Consumption_Df, orient= 'h')
ax = sns.barplot(x= "Power Consumption(Hp)", y= "Trajectory Planners" , 
                 data = Planners_Consumption_Df, 
                 palette=['Black', 'Red','Green', 'Blue'],
                 orient= 'h', ax=axes[0])


ax.bar_label(ax.containers[0])
#ax.set_ylim(0,60)

ax.invert_yaxis()



#PLOT Brake Dissipated friction wheel Power

Op_friction_hp = Mean_Op_brake_power
Astar_friction_hp = Mean_Astar_brake_power
Lattice_friction_hp = Median_Lattice_brake_power
Public_Road_friction = Mean_Public_Road_brake_power

Data_Dictionary_brake = {'A*':Astar_friction_hp, 'Lattice':Lattice_friction_hp, 'Op':Op_friction_hp,  'Public_Road':Public_Road_friction }
data_items_brake = Data_Dictionary_brake.items()
data_List_brake = list(data_items_brake)
Planners_Friction_Df = pd.DataFrame(data_List_brake)
Planners_Friction_Df.columns = ["Trajectory Planners", "Power Consumption Friction(Hp)"]

Planners_Friction_Df.to_csv('df_brake_csv', index=False)


sns.set_theme(style="whitegrid")
#Plot Vertically
#ax = sns.barplot(x= "Trajectory Planners", y="Power Consumption(Hp)" , data = Planners_Consumption_Df)
#Plot Horizontally
#ax = sns.barplot(x= "Power Consumption Friction(Hp)", y= "Trajectory Planners" , data = Planners_Friction_Df, orient= 'h')
ax = sns.barplot(x= "Power Consumption Friction(Hp)", y= "Trajectory Planners" , 
                 data = Planners_Friction_Df, 
                 orient= 'h', 
                 palette=['Black', 'Red', 'Green', 'Blue'],
                 ax=axes[1])

ax.set(xlabel='Power Consumption: Acceleration(Up) | Friction(Bottom)')

ax.bar_label(ax.containers[0])
#ax.set_ylim(0,60)

ax.invert_yaxis()


plt.show()



