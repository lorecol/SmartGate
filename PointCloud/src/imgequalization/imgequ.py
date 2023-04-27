import numpy as np


# create our own histogram function
def get_histogram(image, bins):
    # array with size of bins, set to zeros
    histogram = np.zeros(bins)

    # loop through pixels and sum up counts of pixels
    for pixel in image:
        histogram[pixel] += 1

    # return our final result
    return histogram

# create our cumulative sum function
def cumsum(a):
    a = iter(a)
    b = [next(a)]
    for i in a:
        b.append(b[-1] + i)
    return np.array(b)

def equalizethis(img):
    img = np.asarray(img)*20/2.303
    img = np.array(list(map(np.int_, img)))
    flat = img.flatten()

    hist = get_histogram(flat, 65536)

    # re-normalize cumsum values to be between 0-255
    cs = cumsum(hist)

    # numerator & denomenator
    nj = (cs - cs.min()) * 65535
    N = cs.max() - cs.min()

    # re-normalize the cdf
    cs = nj / N
    cs = cs.astype('uint16')
    img_new = cs[flat]
    img_new = np.reshape(img_new, img.shape)

    return img_new