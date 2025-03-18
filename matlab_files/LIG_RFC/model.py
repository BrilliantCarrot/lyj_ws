import torch 
import torch.nn as nn

'''
    - Time-series Classification 
    - Transformer-based model
    - Only Encoder
'''

device = torch.device('cuda:0')

class PositionalEncoding(nn.Module):
    def __init__ (self, d_model, dropout, max_len=5000):
        super().__init__()
        self.dropout = nn.Dropout(dropout)
        
        E_i = torch.arange(0, d_model, 2).reshape(1,-1)
        pos = torch.arange(0, max_len).reshape(-1,1)
        
        pos_embedding = torch.zeros((max_len, d_model))
        pos_embedding[:,0::2] = torch.sin( pos / torch.pow(10000, E_i / d_model) )
        pos_embedding[:,1::2] = torch.cos( pos / torch.pow(10000, E_i / d_model) )
        
        self.pos_embedding = pos_embedding.to(device)
        
    def forward (self, x) : 
        _, seq_len, __ = x.size()
        x = x + self.pos_embedding[:seq_len,:]
        
        return self.dropout(x)


class custom_model(nn.Module):
    def __init__ (self, d_model, dropout, iw, nhead, n_layer, n_feat, n_class):
        super().__init__()
        
        self.PE = PositionalEncoding(d_model, dropout)
        
        self.embedding = nn.Sequential(
            nn.Linear(n_feat, d_model//2), 
            nn.ReLU(),
            nn.Linear(d_model//2, d_model)
        )
        
        custom_encoder   = nn.TransformerEncoderLayer(d_model=d_model, 
                                                      nhead=nhead, dropout=dropout,
                                                      batch_first=True)
        self.transformer = nn.TransformerEncoder(custom_encoder, num_layers=n_layer)
        
        self.projected = nn.Sequential(
            nn.Linear(d_model, d_model//2), 
            nn.ReLU(), 
            nn.Linear(d_model//2, n_feat) )
        
        
        self.out = nn.Linear(n_feat*iw, n_class)

        self.iw     = iw
        self.n_feat = n_feat
        
    
    def forward(self, src):
        src = self.embedding(src)
        src = self.PE(src)

        output = self.transformer(src)
        output = self.projected(output)
        output = output.reshape(-1, self.iw * self.n_feat)
        output = self.out(output)
        
        return output