import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.svm import SVC


# Import dataset from npz file
with np.load('/Users/jvelasquez/Virginia_Tech/Spring_2023/ECE_4806/pedestal_svm/upright_fallen_dataset.npz') as data:
    print(data.files)
    images = data['images']
    labels = data['labels']


# Split dataset into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(images, labels, test_size=0.2, random_state=42)

# Preprocess the images
X_train = X_train.astype('float32') / 255
X_test = X_test.astype('float32') / 255

# Flatten the images
X_train = X_train.reshape(X_train.shape[0], -1)
X_test = X_test.reshape(X_test.shape[0], -1)

# Use GridSearchCV to find the best parameters for the SVM classifier
from sklearn.model_selection import GridSearchCV

# Define the parameter grid to search
param_grid = {
    "kernel": ["linear", "poly", "rbf"],
    "C": [0.1, 1, 10],
    "gamma": ["scale", "auto"],
    "class_weight": [None, "balanced"]
}

# Create an SVM classifier object
svm = SVC()

# Use GridSearchCV to find the best combination of hyperparameters, running verbose
grid_search = GridSearchCV(svm, param_grid, cv=5, n_jobs=-1, verbose=4, return_train_score=True, pre_dispatch=8)
grid_search.fit(X_train, y_train)

# Print the best hyperparameters found
print("Best parameters:", grid_search.best_params_)