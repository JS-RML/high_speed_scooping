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
DEVICE = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print("PyTorch running on device: {}".format(DEVICE))

# directories
DATASET_DIR = 'dataset/'
MODEL_DIR = 'model/'

# data parameters
SEQ_SRT_OFFSET, SEQ_END_OFFSET = -20, 20 # sequence boundary offset
DATA_TYPE = ['L0', 'L1', 'R0', 'R1', 'L0_cur', 'L1_cur', 'R0_cur', 'R1_cur'] # motor angles and current

# model parameters
INPUT_DIM = len(DATA_TYPE)
HIDDEN_DIM = 8
OUTPUT_DIM = 1

# training parameter
EPOCH = 1000
BATCH_SIZE = 3


class LSTMClassifier(nn.Module):
    def __init__(self, input_dim, hidden_dim, output_dim):
        super().__init__()
        self.hidden_dim = hidden_dim
        self.lstm = nn.LSTM(input_dim, hidden_dim, batch_first=True)
        self.fc = nn.Linear(hidden_dim, output_dim)

    def init_hidden(self, input_):
        h0 = torch.zeros(1, input_.size(0), self.hidden_dim).to(DEVICE)
        c0 = torch.zeros(1, input_.size(0), self.hidden_dim).to(DEVICE)
        return (h0, c0)

    def forward(self, input_):
        h0, c0 = self.init_hidden(input_)
        lstm_out, (hn, cn) = self.lstm(input_, (h0, c0))
        y = self.fc(lstm_out[:,-1,:])
        # output = F.log_softmax(y, dim=0)
        output = F.sigmoid(y)
        return output

def get_sequence(packed_data):
    srt_idx, end_idx = get_joint_angle_boundary(packed_data, SEQ_SRT_OFFSET, SEQ_END_OFFSET) # get boundary of data during contact interaction
    sequence = []
    for idx in range(srt_idx, end_idx):
        vector = []
        for t in range(len(DATA_TYPE)):
            vector.append(packed_data[DATA_TYPE[t]][idx])
        sequence.append(vector)
    return sequence

class MotorDataset(Dataset):
    # load sequential data of motor's angle and current from json files
    def __init__(self, dataset_names):
        self.data = []
        self.label = []
        for set_name in dataset_names: # loop for multiple datasets of different objects
            dataset_dir = DATASET_DIR + set_name

            labels = [0, 1] # 0: grasping failed, 1: grasping succeed
            for lab in labels:
                dataset_label_dir = dataset_dir + str(labels[lab]) # dataset with label
                files = os.listdir(dataset_label_dir)
                for f_name in files:
                    f = open(dataset_label_dir + '/' + f_name)
                    packed_data = json.load(f)
                    self.data.append(get_sequence(packed_data))
                    self.label.append([lab])

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        return torch.from_numpy(np.array(self.data[idx])).float().to(DEVICE) , self.label[idx]

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
    padded_seq = torch.zeros(len(sequences), max(length), INPUT_DIM).float().to(DEVICE)
    for i, seq in enumerate(sequences):
        end = length[i]
        padded_seq[i][:end] = seq
    return padded_seq, torch.from_numpy(np.array(length)), torch.from_numpy(np.array(label)).float().to(DEVICE)

def train():
    datasets_name = ['domino']
    model_save_path = MODEL_DIR + '_'.join(datasets_name) + '.pth'

    datasets = MotorDataset(datasets_name)
    data_loader = DataLoader(datasets, batch_size=BATCH_SIZE, shuffle=True, collate_fn=collate_fn)

    model = LSTMClassifier(INPUT_DIM, HIDDEN_DIM, OUTPUT_DIM).to(DEVICE)
    # loss_function = nn.NLLLoss()
    loss_function = nn.BCELoss()
    optimizer = optim.Adam(model.parameters(),lr = 1e-3)

    best_acc = 0
    for epoch in range(EPOCH):

        # training 
        model.train() # set training mode
        losses = []
        for i, (batched_sequences, length, batched_labels) in enumerate(data_loader):
            model.zero_grad() # clear gradients
            
            score = model(batched_sequences) # run model forward function

            loss = loss_function(score, batched_labels) # compute loss 
            loss.backward() # compute loss gradients

            optimizer.step() # update model parameters

            losses.append(float(loss))
        avg_loss = np.mean(losses)

        # validation
        model.eval() # set evaluation mode
        correct, total = 0, 0
        for i in range(len(datasets)):
            sequence, label = datasets[i]
            input = torch.unsqueeze(sequence, 0) # add one more dimension

            output = model(input)
            result = (output>0.5).float()
            correct += (result[0]==label[0]).float().item()
            total += 1.0
        acc = correct/total * 100

        if epoch % 100 == 0:
            print("Epoch {}/{}: Loss = {:.3f}, Accuracy = {:.3f}%".format(epoch+1, EPOCH, avg_loss, acc))

        if acc > best_acc:
            best_acc = acc
            torch.save(model.state_dict(), model_save_path)
            print("Epoch {}: best model saved with accuracy: {:.3f}%".format(epoch, acc))


def classify(packed_data, trained_model_name):
    '''
    Using the trained model to classify the signal from the packed_data in data_logger.py
    Return: Grasping result, 1: grasped, 0: grasp failed
    '''
    model = LSTMClassifier(INPUT_DIM, HIDDEN_DIM, OUTPUT_DIM).to(DEVICE)
    model.load_state_dict(torch.load(model_name + '.pth'))
    model.eval()

    sequence = get_sequence(packed_data)
    input = torch.unsqueeze(sequence, 0)
    output = model(input)
    return (output>0.5).int().item()

if __name__ == "__main__":
    train()
