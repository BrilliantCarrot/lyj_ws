function Noisy_data = Gaussian_Noise(data, sigma)
rng('shuffle')
noise = sigma * randn(size(data)); 
Noisy_data = data + noise; 

end