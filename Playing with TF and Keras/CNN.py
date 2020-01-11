#for an indroduction in CNN read 
#https://towardsdatascience.com/introduction-to-convolutional-neural-networks-cnn-with-tensorflow-57e2f4837e18
#https://www.tensorflow.org/beta/tutorials/images/intro_to_cnns

import tensorflow as tf
from tensorflow.keras import datasets, layers, models

# Two Tuples
(train_images, train_labels),(test_images,test_label) = datasets.mnist.load_data()


print (train_images.shape) #(60000, 28, 28) 60000 images 28*28
print (train_labels.shape) #(60000,1 )

train_images = train_images.reshape((60000, 28,28,1))
test_images = test_images.reshape((10000,28,28,1))

#normalize pixel values between 0 and 1 
train_images, test_images = train_images/255.0 , test_images/255.0

# Input to the CNN (image_height, image_width, color_channels)

model = models.Sequential()
model.add(layers.Conv2D (32, (3,3), activation = 'relu', input_shape =(28,28,1)))
model.add(layers.MaxPooling2D((2,2)))
model.add(layers.Conv2D(64, (3,3), activation = 'relu'))
model.add(layers.MaxPooling2D((2,2)))
model.add(layers.Conv2D(64, (3,3), activation = 'relu'))
model.add(layers.Flatten())
model.add(layers.Dense(64, activation = 'relu'))
model.add(layers.Dense(10, activation = 'softmax'))

model.summary()

model.compile(optimizer = 'adam', loss = 'sparse_categorical_crossentropy', metrics = ['accuracy'])
model.fit(train_images, train_labels, epochs=5)

test_loss, test_acc = model.evaluate(test_images, test_labels)

print(test_acc)


