import torch

class AMASSPreprocess:
    def __init__(self):
        pass

    def preprocess(self):
        ly = 0.0

        rot_ckpt = torch.load("../vrot.pt", map_location="cpu")
        acc_ckpt = torch.load("../vacc.pt", map_location="cpu")

        # 첫 번째 몇 개 요소만 보기

        for rot, acc in zip(rot_ckpt[0], acc_ckpt[0]):
            print('rot', rot.shape)
            print('acc', acc.shape)
            print('-'*30)

            # print('-'*30)


AMASSPreprocess().preprocess()
