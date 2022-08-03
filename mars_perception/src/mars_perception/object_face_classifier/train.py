import logging
import argparse
import datetime
import sys

import yaml
import os
import torch
from torch import nn
from torch import optim
from torch.utils.data import DataLoader
from torchvision.transforms import ToTensor, Compose, Lambda
from torchvision.datasets import ImageFolder
#from torchvision.datasets import MNIST
from torch.utils.data import random_split
from model import SimpleClassifierNN, SimpleCNN


class Trainer:
    def __init__(self, cfg):
        torch.manual_seed(cfg['seed'])
        cfg_data = cfg["dataset"]
        tfs = Compose(
            [
                ToTensor(),
                Lambda(lambda x: x[:1, :, :]),
            ]
        )
        self.dataset = ImageFolder(cfg_data['data_dir'] + '/'+ cfg_data["dataset_name"] + '_proc', transform=tfs)
        train_dataset, test_dataset = random_split(
            self.dataset,
            (round(0.8 * len(self.dataset)), round(0.2 * len(self.dataset))),
        )

        # train_dataset = MNIST(
        #     root="data",
        #     train=True,
        #     download=True,
        #     transform=ToTensor(),
        # )
        # test_dataset = MNIST(
        #     root="data",
        #     train=False,
        #     download=True,
        #     transform=ToTensor(),
        # )

        cfg_train = cfg["train"]
        self.device = torch.device(cfg_train["device"])
        self.model = SimpleCNN(cfg_data["img_dim"], len(cfg_data["axes"]))
        self.model.to(self.device)
        self.epochs = cfg_train["epochs"]

        self.trainloader = DataLoader(
            train_dataset, cfg_train["batch_size"], shuffle=True
        )
        self.testloader = DataLoader(
            test_dataset, cfg_train["batch_size"], shuffle=True
        )
        self.optimizer = optim.SGD(
            self.model.parameters(), lr=cfg_train["lr"], momentum=0.9
        )
        self.criterion = nn.CrossEntropyLoss()

        # logging
        log_folder = datetime.datetime.now().strftime("log_%m-%d-%Y_%H-%M-%S")
        if not os.path.isdir(".log"):
            os.mkdir(".log")
        self.log_pth = os.path.join(".log", log_folder)
        os.mkdir(self.log_pth)
        with open(self.log_pth + '/config.yml', 'w') as outfile:
            yaml.dump(cfg, outfile, default_flow_style=False)
        logging.basicConfig(
            level=logging.INFO,
            format="%(message)s",
            handlers=[
                logging.FileHandler(f".log/{log_folder}/debug.log"),
                logging.StreamHandler(sys.stdout),
            ],
        )
        self.chpt = 0

    def train(self):
        size = len(self.trainloader.dataset)
        for batch, data in enumerate(self.trainloader):
            # get the inputs; data is a list of [inputs, labels]
            inputs, labels = data
            inputs = inputs.to(self.device)
            labels = labels.to(self.device)

            # zero the parameter gradients
            self.optimizer.zero_grad()

            # forward + backward + optimize
            outputs = self.model(inputs)
            loss = self.criterion(outputs, labels)
            loss.backward()
            self.optimizer.step()

            if batch % 2 == 0:
                loss, current = loss.item(), batch * len(inputs)
                logging.info(f"loss: {loss:>7f}  [{current:>5d}/{size:>5d}]")

    def test(self):
        num_batches = len(self.testloader)
        self.model.eval()
        test_loss = 0
        with torch.no_grad():
            for data in self.testloader:
                # get the inputs; data is a list of [inputs, labels]
                inputs, labels = data
                inputs = inputs.to(self.device)
                labels = labels.to(self.device)
                outputs = self.model(inputs)
                test_loss += self.criterion(outputs, labels).item()
        test_loss /= num_batches
        logging.info(f"Test Error: \n Avg loss: {test_loss:>8f} \n")

    def train_test(self):
        for i in range(self.epochs):
            logging.info(f"Epoch {i+1}\n-------------------------------")
            self.train()
            self.test()
            if i % 10:
                self.checkpoint()

    def checkpoint(self, name=None):
        if name is None:
            name = self.chpt
            self.chpt += 1
        torch.save(self.model.state_dict(), self.log_pth + f"/chpt_{name}.pth")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-cfg_path", help="path to config file", type=str, default='./config.yml')
    
    args = vars(parser.parse_args())
    with open(args['cfg_path'], "r") as f:
        params = yaml.safe_load(f)

    trainer = Trainer(params)
    trainer.train_test()
    trainer.checkpoint("final")