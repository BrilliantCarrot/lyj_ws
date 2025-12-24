function filtered_signal = Gaussian_Filter(data, sigma)
    filter_size = 2 * ceil(3*sigma) + 1;
    x = linspace(-ceil(3*sigma), ceil(3*sigma), filter_size);
    gaussian_mask = exp(-x .^ 2 / (2 * sigma ^ 2));
    gaussian_mask = gaussian_mask / sum(gaussian_mask);
    filtered_signal = conv(data, gaussian_mask, 'same');
end