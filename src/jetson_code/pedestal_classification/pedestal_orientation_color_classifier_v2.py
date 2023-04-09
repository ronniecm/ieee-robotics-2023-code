# Version 2 of the pedestal orientation and color classifier, creates training, validation, and test datasets
# from a single dataset instead of two, like in version 1

import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import torch.nn.functional as F
import os
import matplotlib.pyplot as plt
from torchinfo import summary
import torchvision.transforms as transforms
from torch.utils.tensorboard import SummaryWriter
from torch.utils.data import Dataset, TensorDataset, DataLoader

from ImageClassifierNets import LightweightCNN, EarlyStopper, CustomTensorDataset

from datetime import datetime

def main(): 

    torch.set_num_threads(1)

    model = LightweightCNN()
    optimizer = optim.SGD(model.parameters(), lr=0.001, momentum=0.9)
    criterion = nn.CrossEntropyLoss()

    # Check to see what device is available
    if torch.backends.mps.is_available():
        device = torch.device("mps")
        print("Using the Apple MPS backend")
    elif torch.cuda.is_available():
        device = torch.device("cuda")
        print("Using the CUDA backend")
    else:
        device = torch.device("cpu")
        print("Using the CPU backend")
        
    model.to(device)
    print(model)

    # Input a dummy tensor to the model
    dummy_input = torch.randn(1, 3, 120, 160, device=device)
    out = model(dummy_input)
    print(out.shape)
    print(out)
        
    # Import training and validation dataset from npz file
    with np.load('/Users/jvelasquez/Virginia_Tech/Spring_2023/ECE_4806/ieee-robotics-2023-code/src/jetson_code/pedestal_classification/pedestal_color_orientation_dataset_v3.npz') as data:
        print(data.files)
        images = data['images']
        labels = data['labels']

    # Preprocess the images
    images = images.astype('float32') / 255
    print(images[0].shape)

    # Make labels shape (n, 1)
    #labels.reshape(labels.shape[0], 1)

    # Convert the numpy arrays to PyTorch tensors
    images_tensor = torch.from_numpy(images)
    labels_tensor = torch.from_numpy(labels)

    # Define the image transformations
    transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.RandomHorizontalFlip(),s
        transforms.RandomVerticalFlip(),
        transforms.RandomRotation(45),
        # transforms.ColorJitter(brightness=0.25),
        # transforms.RandomPerspective(),
        transforms.ToTensor()
    ])

    # Split the dataset into training, validation, and testing sets
    train_split = 0.90
    train_size = int(train_split * len(images_tensor))
    val_size = len(images_tensor) - train_size
    train_indices, temp_indices = torch.utils.data.random_split(torch.arange(len(images_tensor)), [train_size, val_size])

    # Split the validation set into validation and test sets
    val_split = 0.5
    val_size = int(val_split * len(temp_indices))
    test_size = len(temp_indices) - val_size
    val_indices, test_indices = torch.utils.data.random_split(temp_indices, [val_size, test_size])

    # Create the datasets
    train_images_tensor = images_tensor[train_indices]
    train_labels_tensor = labels_tensor[train_indices]

    val_images_tensor = images_tensor[val_indices]
    val_labels_tensor = labels_tensor[val_indices]

    test_images_tensor = images_tensor[test_indices]
    test_labels_tensor = labels_tensor[test_indices]

    train_dataset = CustomTensorDataset(tensors=(train_images_tensor, train_labels_tensor), transform=transform, device=device)
    val_dataset = CustomTensorDataset(tensors=(val_images_tensor, val_labels_tensor), transform=transform, device=device)

    test_dataset = TensorDataset(test_images_tensor, test_labels_tensor)

    # Create the dataloaders
    batch_size = 64

    train_loader = torch.utils.data.DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=16, pin_memory=True, prefetch_factor=256, multiprocessing_context='fork')
    val_loader = torch.utils.data.DataLoader(val_dataset, batch_size=batch_size, shuffle=False, num_workers=16, pin_memory=True, prefetch_factor=256, multiprocessing_context='fork')
    test_loader = torch.utils.data.DataLoader(test_dataset, batch_size=batch_size, shuffle=False, num_workers=8, pin_memory=True, prefetch_factor=64)

    summary(model, input_size=(batch_size, 3, 120, 160), verbose=2, device=device)

    num_epochs = 500

    # Create SummaryWriter for TensorBoard, create a new directory for each run by appending the current time to the path
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    new_path = f'model_logs/run_{timestamp}'

    # Append current working directory to the path
    new_path = os.path.join(os.getcwd(), new_path)

    # Make the new directory
    os.makedirs(new_path)

    writer = SummaryWriter(log_dir=new_path)

    # Write graph of the model to TensorBoard
    writer.add_graph(model, dummy_input)

    # Initialize lists to store loss and accuracy for each epoch
    train_losses = []
    train_accs = []
    val_losses = []
    val_accs = []
    test_losses = []
    test_accs = []

    # Increment epoch counter
    epochs = 0

    # Define an early stopping object
    early_stopper = EarlyStopper(patience=50, min_delta=0.0002)

    for epoch in range(num_epochs):
        # Train
        model.train()
        train_loss = 0
        correct_train = 0
        total_train = 0
        for i, data in enumerate(train_loader):

            # Send the data to mps
            images, labels = data[0].to(device), data[1].to(device)

            optimizer.zero_grad()
            outputs = model(images)
            
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            train_loss += loss.item()
            _, predicted = torch.max(outputs.data, 1)
            
            total_train += labels.size(0)
            correct_train += (predicted == labels).sum().item()
            
        train_loss /= len(train_loader)
        train_losses.append(train_loss)

        train_acc = 100 * correct_train / total_train
        train_accs.append(train_acc)

        # Write to TensorBoard for logging
        # writer.add_scalar('Loss/train', train_loss, epoch)
        # writer.add_scalar('Accuracy/train', train_acc, epoch)
        
        # Validate
        model.eval()
        val_loss = 0
        correct_val = 0
        total_val = 0
        with torch.no_grad():
            for i, data in enumerate(val_loader):
                # Send the data to mps
                images, labels = data[0].to(device), data[1].to(device)
                outputs = model(images)
                loss = criterion(outputs, labels)
                val_loss += loss.item()
                _, predicted = torch.max(outputs.data, 1)
                total_val += labels.size(0)
                correct_val += (predicted == labels).sum().item()

        val_loss /= len(val_loader)
        val_losses.append(val_loss)
        val_acc = 100 * correct_val / total_val
        val_accs.append(val_acc)

        # Write to TensorBoard for logging
        # writer.add_scalar('Loss/val', val_loss, epoch)
        # writer.add_scalar('Accuracy/val', val_acc, epoch)
        
        # Test
        model.eval()
        test_loss = 0
        correct_test = 0
        total_test = 0
        with torch.no_grad():
            for i, data in enumerate(test_loader):
                # Send the data to mps
                images, labels = data[0].to(device), data[1].to(device)
                outputs = model(images)
                loss = criterion(outputs, labels)

                test_loss += loss.item()
                _, predicted = torch.max(outputs.data, 1)
                total_test += labels.size(0)
                correct_test += (predicted == labels).sum().item()
        test_loss /= len(test_loader)
        test_losses.append(test_loss)
        test_acc = 100 * correct_test / total_test
        test_accs.append(test_acc)

        # Write to TensorBoard for logging
        # writer.add_scalar('Loss/test', test_loss, epoch)
        # writer.add_scalar('Accuracy/test', test_acc, epoch)

        writer.add_scalars(f'loss', {
        'train': train_loss,
        'val': val_loss,
        'test': test_loss,
        }, epoch)
        
        writer.add_scalars(f'accuracy', {
        'train': train_acc,
        'val': val_acc,
        'test': test_acc,
        }, epoch)
        
        # Print statistics
        print('Epoch [{}/{}], Train Loss: {:.4f}, Train Acc: {:.2f}%, Val Loss: {:.4f}, Val Acc: {:.2f}%, Test Loss: {:.4f}, Test Acc: {:.2f}%'.format(epoch+1, num_epochs, train_loss, train_acc, val_loss, val_acc, test_loss, test_acc))

        # Increment epoch counter
        epochs += 1

        # Check if the model should stop early
        if early_stopper.early_stop(val_loss):             
            break


    # Plot loss and accuracy curves for training, validation, and testing sets
    plt.plot(range(epochs), train_losses, label='Training Loss')
    plt.plot(range(epochs), val_losses, label='Validation Loss')
    plt.plot(range(epochs), test_losses, label='Testing Loss')
    plt.legend()
    plt.title('Loss Curves')
    plt.xlabel('Epoch')
    plt.ylabel('Loss')
    plt.grid()

    # Save the loss curves to the TensorBoard directory
    plt.savefig(os.path.join(new_path, 'loss_curves.png'))
    plt.show()

    plt.plot(range(epochs), train_accs, label='Training Accuracy')
    plt.plot(range(epochs), val_accs, label='Validation Accuracy')
    plt.plot(range(epochs), test_accs, label='Testing Accuracy')
    plt.legend()
    plt.title('Accuracy Curves')
    plt.xlabel('Epoch')
    plt.ylabel('Accuracy')
    plt.grid()

    # Save the accuracy curves to the TensorBoard directory
    plt.savefig(os.path.join(new_path, 'accuracy_curves.png'))
    plt.show()
    
    # Close the writer
    writer.close()

    # Append the current time to the model name
    curr_time = timestamp
    model_name = 'lightweight_net_color_orientation' + '_' + curr_time + '.pth'

    # Save the model
    torch.save(model.state_dict(), model_name)

if __name__ == '__main__':
    main()