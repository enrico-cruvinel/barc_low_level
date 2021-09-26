import turtle
import matplotlib.pyplot as plt
import numpy as np

#COLORS = ['g','g','r','c','m','y','k']

class TurtleFig():

	def __init__(self, width = 800, height = 600):
		self.window = turtle.Screen()
		self.window.setup(width,height)
		self.objects = []
		self.width = width
		self.height = height


	def createObject(self,color):
		object = turtle.Turtle()
		object.shape('square')
		object.color(color)
		object.penup()
		object.goto(0,0)
		object.speed(1)
		self.objects.append(object)
		return len(self.objects) - 1 # returns the object adress(object_id) so it can be accessed for moving 

	def moveObject(self,object_id,x,y):
		object = self.objects[object_id]
		object.setx(x)
		object.sety(y)
		self.check_bounds(object_id)
		
		return

	def rotateobject(self,object_id,angle): # angle in radians
		object = self.objects[object_id]
		object.setheading(np.rad2deg(angle))
		
		return
	
	def check_bounds(self, object_id):
		object = self.objects[object_id]
		if abs(object.position()[0]) >= self.width/2:
			object.setx(0)
		if abs(object.position()[1]) >=  self.height/2:
			object.sety(0)

		return