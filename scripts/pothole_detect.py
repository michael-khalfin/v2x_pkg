#!/usr/bin/env python3

# import rospy
# from std_msgs.msg import Empty
# from dynamic_reconfigure.server import Server
# from v2x.cfg import V2XConfig

import tensorflow as tf
import cv2
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

data_x = []
data_y = []

def get_image(file_path):
    img = cv2.imread(file_path)
    img = cv2.resize(img,(100,100))
    img = img.astype("float32")
    img /= 255
    return img

cone_imgs = list(Path("../cones").glob('*.png'))
cone_imgs = [str(x) for x in cone_imgs]
for img_path in cone_imgs:
    data_x.append(get_image(img_path))
    data_y.append([1,0])

notcone_imgs = list(Path("../notcones").glob('*.png'))
notcone_imgs = [str(x) for x in notcone_imgs]
for img_path in notcone_imgs:
    data_x.append(get_image(img_path))
    data_y.append([0,1])

data_x = np.array(data_x)
data_y = np.array(data_y)

x_train = data_x[:15]
print(x_train.shape)
x_train = np.append(x_train, data_x[22:35], axis=0)
y_train = data_y[:15]
y_train = np.append(y_train, data_y[22:35], axis=0)

x_test = data_x[15:22]
x_test = np.append(x_test, data_x[35:], axis=0)
y_test = data_y[15:22]
y_test = np.append(y_test, data_y[35:], axis=0)


test_size = len(x_test)

x_train = tf.convert_to_tensor(x_train)
y_train = tf.convert_to_tensor(y_train)
x_test = tf.convert_to_tensor(x_test)
y_test = tf.convert_to_tensor(y_test)

print(x_train.shape)
print(y_train.shape)
print(x_test.shape)
print(y_test.shape)

model = tf.keras.Sequential()
model.add(tf.keras.layers.Dense(8))
model.add(tf.keras.layers.Dense(1))
model.compile(optimizer='sgd', loss='mse', metrics=['mae'])
model.fit(x_train,y_train,batch_size=1, epochs=10)

x = np.expand_dims(x_train[0], axis=0)
pred = model.predict(x_train[0])
print("predicted " + str(pred[0][0]))
# for i in range(test_size):
#     x = np.expand_dims(x_test[i], axis=0)
#     y = y_test[i][0]
#     pred = model.predict(x)
#     pred = pred[0][0]
#     print(f'Feature {i}\nModel predicted: {pred}. Actual label: {y}\nLoss of: {abs(pred-y)}')

test_perf = model.fit(x_test,y_test,batch_size=1, epochs=10)

plt.plot(test_perf.history['loss'], label='loss')
plt.plot(test_perf.history['mae'], label='mae')
plt.ylabel('loss')
plt.legend()
plt.show()

# def input_fn(features, labels, training=True, batch_size=10):
#     dataset = tf.data.Dataset.from_tensor_slices((dict(features), labels))

#     if training:
#         dataset = dataset.shuffle(40).repeat()

#     return dataset.batch(batch_size)

# features = []
# for ind in range(len(data_x)):
#     features.append(tf.feature_column.numeric_column(key=str(ind)))

# classifier = tf.estimator.LinearClassifier(feature_columns=features, n_classes=2)
# classifier.train(input_fn=lambda: input_fn(data_x, data_y, training=True), steps=5000)