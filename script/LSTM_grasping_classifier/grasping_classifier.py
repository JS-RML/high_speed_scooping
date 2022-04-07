import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import TensorDataset, Dataset, DataLoader
import numpy as np
import os
import json
import sys
sys.path.append("..")
from data_logger import get_joint_angle_boundary

torch.manual_seed(1)
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print("PyTorch running on device: {}".format(device))

# model paremeters
INPUT_DIM = 8
HIDDEN_DIM = 8
OUTPUT_DIM = 1

# training parameter
EPOCH = 1000


class LSTMClassifier(nn.Module):
    def __init__(self, input_dim, hidden_dim, output_dim):
        super().__init__()
        self.hidden_dim = hidden_dim
        self.lstm = nn.LSTM(input_dim, hidden_dim, batch_first=True)
        self.fc = nn.Linear(hidden_dim, output_dim)

    def init_hidden(self, input_):
        h0 = torch.zeros(1, input_.size(0), self.hidden_dim).to(device)
        c0 = torch.zeros(1, input_.size(0), self.hidden_dim).to(device)
        return (h0, c0)

    def forward(self, input_):
        h0, c0 = self.init_hidden(input_)
        lstm_out, (hn, cn) = self.lstm(input_, (h0, c0))
        y = self.fc(lstm_out[:,-1,:])
        # output = F.log_softmax(y, dim=0)
        output = F.sigmoid(y)
        return output

def create_DataLoader():
    '''
    Load dataset from json in /dataset
    Return: Dictionary of two PyTorch DataLoader type: 'train', 'valid'
    '''
    selected_dataset = ['domino1', 'domino0'] # folder name of dataset in /dataset, 1: grasp success, 0: grasp failed
    


    return DataLoader()

class MotorDataset(Dataset):
    # load sequential data of motor's angle and current from json files
    def __init__(self, dataset_names):
        self.data = []
        self.label = []
        for set_name in dataset_names: # loop for multiple datasets of different objects
            dataset_dir = 'dataset/' + set_name

            labels = [0, 1] # 0: grasping failed, 1: grasping succeed
            for lab in labels:
                dataset_label_dir = dataset_dir + str(labels[lab]) # dataset with label
                files = os.listdir(dataset_label_dir)
                for f_name in files:
                    f = open(dataset_label_dir + '/' + f_name)
                    json_data = json.load(f)
                    srt_idx, end_idx = get_joint_angle_boundary(json_data,-20,20) # get boundary of data during contact interaction
                    sequence = []
                    for idx in range(srt_idx, end_idx):
                        data_type = ['L0', 'L1', 'R0', 'R1', 'L0_cur', 'L1_cur', 'R0_cur', 'R1_cur']
                        INPUT_DIM = len(data_type) # update input dimension if changed
                        vector = []
                        for t in range(len(data_type)):
                            vector.append(json_data[data_type[t]][idx])
                        sequence.append(vector)
                    self.data.append(sequence)
                    self.label.append([lab])

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        return torch.from_numpy(np.array(self.data[idx])).to(device) , self.label[idx]

def collate_fn(data):
    '''  
	Build a custom collate_fn rather than using default collate_fn,
	as the size of every sentence is different and merging sequences (including padding) 
	is not supported in default. 
	Args:
		data: list of tuple (training sequence, label) with the number of batch size
	Return:
		padded_seq - Padded Sequence, tensor of shape (batch_size, padded_length)
		length - Original length of each sequence(without padding), tensor of shape(batch_size)
		label - tensor of shape (batch_size)
    '''
    #sorting is important for usage pack padded sequence (used in model). It should be in decreasing order.
    data.sort(key=lambda x: len(x[0]), reverse=True)
    sequences, label = zip(*data)
    length = [len(seq) for seq in sequences] # find length of each sequence
    padded_seq = torch.zeros(len(sequences), max(length), INPUT_DIM).float().to(device)
    for i, seq in enumerate(sequences):
        end = length[i]
        padded_seq[i][:end] = seq
    return padded_seq, torch.from_numpy(np.array(length)), torch.from_numpy(np.array(label)).float().to(device)

def train():
    train_dataset = MotorDataset(['domino'])
    data_loader = DataLoader(train_dataset, batch_size = 3, collate_fn = collate_fn)

    model = LSTMClassifier(INPUT_DIM, HIDDEN_DIM, OUTPUT_DIM).to(device)
    # loss_function = nn.NLLLoss()
    loss_function = nn.BCELoss()
    optimizer = optim.Adam(model.parameters(),lr = 1e-3)

    epoch_loss = []
    for epoch in range(EPOCH):
        losses = []
        for i, (batched_sequences, length, batched_labels) in enumerate(data_loader):

            model.zero_grad() # clear gradients

            score = model(batched_sequences) # run model forward function

            loss = loss_function(score, batched_labels) # compute loss 
            loss.backward() # compute loss gradients
            optimizer.step() # update model parameters

            losses.append(float(loss))
        avg_loss = np.mean(losses)
        epoch_loss.append(avg_loss)

        if epoch % 100 == 0:
            print("Epoch {} / {}: Loss = {:.3f}".format(epoch+1, EPOCH, avg_loss))


if __name__ == "__main__":
    train()
