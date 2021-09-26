import turtle
import matplotlib.pyplot as plt
import numpy as np

COLORS = ['g','g','r','c','m','y','k']

class TurtleFig():

	def __init__(self):
		self.window = turtle.Screen()
		self.window.setup(800,600)
		self.fig = None
		self.axs = None
		self.agents = []
		self.graph_rate = 0.01

	def graphsHandler(self,num_graphs,graphs_dict):
		self.fig, self.axs = plt.subplots(num_graphs, sharex=True)
		i = 0
		for a in self.axs:
			a.set(xlabel= graphs_dict[i]['xlab'],ylabel= graphs_dict[i]['ylab'])
			i += 1

	def setGraphRate(self,dt):
		self.graph_rate = dt

	def setGraphColors(self):
		pass

	def updateGraphs(self,graph_values):
		i = 0
		for a in self.axs:
			a.plot(graph_values[i][0],graph_values[i][1],'tab:red')
			i += 1
			plt.pause(self.graph_rate)

	def showGraphs(self,graph_values,batch_num = 0):
		i = 0
		for a in self.axs:
			a.plot(graph_values[i][0],graph_values[i][1],'tab:red')
			i += 1
		plt.show()

	def createAgent(self,color):
		agent = turtle.Turtle()
		agent.shape('square')
		agent.color(color)
		agent.penup()
		agent.goto(0,0)
		agent.speed(0)
		self.agents.append(agent)
		return len(self.agents) - 1 # returns the agent adress(ID) so it can be accessed for moving 

	def moveAgent(self,ID,x,y):
		agent = self.agents[ID]
		agent.setx(x)
		agent.sety(y)
		
	def rotateAgent(self,ID,angle): # angle in radians
		agent = self.agents[ID]
		agent.setheading(np.rad2deg(angle))