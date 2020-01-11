import numpy as np
import xlrd
import math
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import tensorflow as tf
from tensorflow.keras import datasets, layers, models


n_features = 3
alpha = 0.5
iterations =500

#graphs 
iterations_for_x = []
accuracy_for_y =[]
weight_for_x = []
b_for_y = []
cost_for_z = []



# loading data from exel
database1 = xlrd.open_workbook("D:/Projects/Git2/Search_Drone/Playing with TF and Keras/fix_dataset3.xlsx")
sheet = database1.sheet_by_index(0)
fileds = []

m = sheet.nrows 
m_traning = math.floor(m*0.75)
m_testing = math.floor(m*0.25)


training_set = np.zeros((m_traning,sheet.ncols))
testing_set = np.zeros((m_testing,sheet.ncols))

for i in range(0,m_traning):
	for j in range(0,sheet.ncols):
		try:
			training_set[i][j] = sheet.cell_value(i,j)
		except:
			fileds.append(sheet.cell_value(i,j))
p=0;

for i in range(m_traning,m):
	p = p + 1
	for j in range(0,sheet.ncols):
		try:
			testing_set[p][j] = sheet.cell_value(i,j)
		except:
			fileds.append(sheet.cell_value(i,j))

training_set = np.delete(training_set,0,0) #Removing the top row
testing_set = np.delete(testing_set,0,0)  #Removing the top row

training_dataset_shape = training_set.shape
testing_data_shape = testing_set.shape

#Input

X_Train = np.transpose(np.delete(training_set,n_features,1))/700 # Removing output column which is last column
X_Test = np.transpose(np.delete(testing_set,n_features,1))/700

#Output

Y_Train = np.reshape(np.transpose(training_set[:,3]), (1,m_traning -1))
Y_Test = np.reshape(np.transpose(testing_set[:,3]), (1,m_testing -1))

#changing inputs to TF fromat
X_Train = tf.transpose(X_Train)
X_Test = tf.transpose(X_Test)
Y_Train = tf.transpose(Y_Train)
Y_Test = tf.transpose(Y_Test)


print("Input for training Shape:    ", X_Train.shape)
print("Input for testing Shape:     ", X_Test.shape)
print("Output for training Shape:   ", Y_Train.shape)
print("Output for training Shape:   ", Y_Test.shape)

model =models.Sequential()

model.add(layers.Dense(3, activation = 'tanh', use_bias = True, input_shape = (3,))) #layer 1
model.add(layers.Dense(100, activation ='tanh', use_bias = True))
model.add(layers.Dense(100, activation ='tanh', use_bias = True))
model.add(layers.Dense(1, activation ='tanh', use_bias = True))

model.summary()

model.compile(optimizer = 'sgd', loss = 'mean_squared_error', metrics = ['accuracy']) # sgd = Stochastic gradient descent optimizer.
model.fit(X_Train, Y_Train, epochs=100, steps_per_epoch =100)




