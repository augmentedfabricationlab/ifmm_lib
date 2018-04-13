'''
Created on 10.11.2017

@author: awalzer
'''


class Bike(object):
    
    #===========================================================================
    def __init__(self, new_speed):
        self.color = "red"
        self.set_speed(new_speed)
        
    def addstuff(self, a , b):
        return a+b

    def changecolor(self, new_color):
        old_color = self.color
        self.color = new_color
        return old_color

    def set_speed(self, new_speed):
        self.speed = new_speed
    
    def get_speed(self):
        return self.speed
    
    def set_speed_slow(self):
        new_speed = "slow"
        self.set_speed(new_speed)

new_speed = "mid"
mybike = Bike(new_speed)
print mybike.speed

oldcolor = mybike.changecolor("green")

print oldcolor
print mybike.color

a = 5
b = 3
result = mybike.addstuff(a, b)
print result

mybike.set_speed("slow")
print mybike.speed

mybike.set_speed("fast")
print mybike.get_speed()

mybike.set_speed_slow()
print mybike.get_speed()

new_speed = "not so fast"
kathrins_bike = Bike(new_speed)
oldcolor = kathrins_bike.changecolor("golden")
print "Kahtrins Bike is " + kathrins_bike.color + " and " + kathrins_bike.get_speed()