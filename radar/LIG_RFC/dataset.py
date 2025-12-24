import torch
import numpy  as np
import pandas as pd

from torch.utils.data        import Dataset
from sklearn.model_selection import train_test_split

km2m = 1000

def nan2mean(data):   #data: (seq_len, n_feat)
    tf = np.all( np.abs(data[:,:2]) < 10 * km2m , axis=1)
    if np.all(tf == False) : 
        return None, None
    else: 
        _mean     = np.mean( data[tf], axis=0 ) 
        data[~tf] = _mean
        return True, data
    

class custom_dataset(Dataset):
    def __init__(self, seq_len, n_sample):
        super().__init__()
    
        feat    = ['px', 'py', 'pz', 'vx', 'vy', 'vz']
        df      = pd.read_csv('C:/Users/leeyj/OneDrive/School/assignment/tot_data_ver2.csv')
        data    = np.array(df[feat])
        # data    = custom_dataset.scaler(data)
        tgt     = df['label'].to_numpy()
        scn_num = df['scn_num']
        d_len   = df.value_counts('scn_num').min() #minimum
        print(d_len)
        d_feat  = len(feat)
        d_n     = int(scn_num.max())

        X   = np.zeros((0, seq_len, d_feat))
        X_o = np.zeros((0, seq_len, d_feat))
        Y   = np.zeros((0, 1))
        Z   = np.zeros((0, 1))

    
        for i_n in range(d_n):
            if i_n % 100 == 0:
                print(i_n, d_n)
            d_tmp = data[scn_num==i_n]
            i_y   = np.array([np.unique(tgt[scn_num==i_n])])
            for i_s in range(n_sample):
                st_idx = np.random.randint(0, d_len - seq_len)
                d_crop = d_tmp[st_idx:st_idx+seq_len,:]
                noise  = np.vstack([*[[*np.random.normal(0, 50, 3), 
                                       *np.random.normal(0, 10, 3)] for _ in range(seq_len)]])
                d_cropb = d_crop + noise
                d_crop  = nan2mean(d_crop + noise)
                if d_crop[0] == None:
                    break
                else:
                    d_crop = d_crop[1]
                    i_x    = d_crop[np.newaxis,:,:]
                    st_idx = np.array([st_idx])[np.newaxis,:]
                    X_o    = np.append(X_o, d_cropb[np.newaxis,:,:], axis=0)
                    X      = np.append(X, i_x, axis=0)
                    Y      = np.append(Y, i_y, axis=0)
                    Z      = np.append(Z, st_idx, axis=0)
        Y            = Y[:,0]
        _            = train_test_split(X, Y, test_size=0.2, random_state=42)
        self.X_train = torch.FloatTensor(_[0].astype(np.float32))
        self.Y_train = torch.LongTensor(_[2].astype(np.float32))
        self.X_test  = torch.FloatTensor(_[1].astype(np.float32))
        self.Y_test  = torch.LongTensor(_[3].astype(np.float32))
        self.X_o     = X_o
        self.X       = X
        self.Y       = Y
        self.Z       = Z
        self.n_dset  = self.X_train.shape[0]
        
        
        
    def scaler(data):
        _max = np.max(data)
        _min = np.min(data)
        
        _data = (data - _min) / (_max - _min)
        
        return _data
        
        
    def __getitem__(self, idx):
        return self.X_train[idx], self.Y_train[idx]
    
    def __len__(self):
        return self.n_dset
    
    
            

    
            
