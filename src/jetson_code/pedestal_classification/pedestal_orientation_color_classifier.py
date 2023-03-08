def main(): 
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
    with np.load('/Users/jvelasquez/Virginia_Tech/Spring_2023/ECE_4806/ieee-robotics-2023-code/src/jetson_code/pedestal_classification/pedestal_color_orientation_dataset.npz') as data:
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

    # Load the testing set
    with np.load('/Users/jvelasquez/Virginia_Tech/Spring_2023/ECE_4806/ieee-robotics-2023-code/src/jetson_code/pedestal_classification/pedestal_color_orientation_test_dataset_v2.npz') as data:
        print(data.files)
        test_images = data['images']
        test_labels = data['labels']

    test_images = test_images.astype('float32') / 255
    #test_labels = test_labels.reshape(test_labels.shape[0], 1)

    # Shuffle test_images and test_labels together
    p = np.random.permutation(len(test_images))
    test_images = test_images[p]
    test_labels = test_labels[p]

    # Convert the numpy arrays to PyTorch tensors
    test_images_tensor = torch.from_numpy(test_images)
    test_labels_tensor = torch.from_numpy(test_labels)

    # Define the image transformations
    transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.RandomHorizontalFlip(),
        transforms.RandomVerticalFlip(),
        transforms.RandomRotation(45),
        transforms.ColorJitter(brightness=0.25),
        # transforms.RandomPerspective(),
        transforms.ToTensor()
    ])

    # Define the dataset using the tensors
    dataset = CustomTensorDataset(tensors=(images_tensor, labels_tensor), transform=transform)

    # Take a portion of the the test images and labels and append them to the dataset
    portion = int(len(test_images_tensor) * 0.75)
    dataset = torch.utils.data.ConcatDataset([dataset, CustomTensorDataset(tensors=(test_images_tensor[:portion], test_labels_tensor[:portion]), transform=transform)])

    # Take the other portion of the test images and labels and use them as the test set
    other_portion = int(len(test_images_tensor) - portion)
    test_set = torch.utils.data.TensorDataset(test_images_tensor[other_portion:], test_labels_tensor[other_portion:])

    # Split the dataset into training, validation, and testing sets
    train_split = 0.85
    train_size = int(train_split * len(dataset))
    val_size = len(dataset) - train_size
    train_set, val_set = torch.utils.data.random_split(dataset, [train_size, val_size])

    # Apply transformations to all images in the training set
    # train_transformed = torch.utils.data.Subset(train_set, train_set.indices)
    # train_transformed.transform = transform

    # Apply transformations to all images in the validation set     
    #val_transformed = torch.utils.data.Subset(val_set, val_set.indices)
    #val_transformed.transform = transform

    # Create the dataloaders
    batch_size = 32
    train_loader = torch.utils.data.DataLoader(train_set, batch_size=batch_size, shuffle=True, num_workers=6, pin_memory=True, prefetch_factor=12, multiprocessing_context='fork')
    val_loader = torch.utils.data.DataLoader(val_set, batch_size=batch_size, shuffle=True, num_workers=6, pin_memory=True, prefetch_factor=12, multiprocessing_context='fork')

    # test_set = torch.utils.data.TensorDataset(test_images_tensor, test_labels_tensor)
    test_loader = torch.utils.data.DataLoader(test_set, batch_size=batch_size, shuffle=False, num_workers=4, pin_memory=True)


    batch_size = 16
    summary(model, input_size=(batch_size, 3, 120, 160), verbose=2, device=device)

    num_epochs = 3000

    # Create SummaryWriter for TensorBoard, create a new directory for each run by checking the number of directories in the model_logs directory
    new_path = 'model_logs/run' + str(len(os.listdir('model_logs')) + 1)

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
    early_stopper = EarlyStopper(patience=100, min_delta=0.0002)

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
    plt.show()

    plt.plot(range(epochs), train_accs, label='Training Accuracy')
    plt.plot(range(epochs), val_accs, label='Validation Accuracy')
    plt.plot(range(epochs), test_accs, label='Testing Accuracy')
    plt.legend()
    plt.title('Accuracy Curves')
    plt.xlabel('Epoch')
    plt.ylabel('Accuracy')
    plt.grid()
    plt.show()

    # Close the writer
    writer.close()

    # Save the model
    torch.save(model.state_dict(), 'lightweight_net_color_orientation_v4.pth')

if __name__ == '__main__':
    main()