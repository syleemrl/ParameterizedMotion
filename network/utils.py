# The MIT License

# Copyright (c) 2017 OpenAI (http://openai.com)

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import numpy as np
import pickle
from IPython import embed
 
class RunningMeanStd(object):
    # https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Parallel_algorithm
    def __init__(self, shape=()):
        self.mean = np.zeros(shape, 'float64')
        self.var = np.ones(shape, 'float64')
        self.count = 1e-4
        self.epsilon = 1e-8
        self.clip = 10

    def update(self, x):
        batch_mean = np.mean(x, axis=0)
        batch_var = np.var(x, axis=0)
        batch_count = x.shape[0]

        self.update_from_moments(batch_mean, batch_var, batch_count)

    def update_from_moments(self, batch_mean, batch_var, batch_count):
        mean, var, count = update_mean_var_count_from_moments(
            self.mean, self.var, self.count, batch_mean, batch_var, batch_count)

        if np.isnan(np.sum(mean)) or np.isnan(np.sum(var)) or np.isnan(np.sum(count)):
            print('input state is NAN')
        else:
            self.mean = mean
            self.var = var
            self.count = count


    def apply(self, x):
        self.update(x)

        x = np.clip((x - self.mean) / np.sqrt(self.var + self.epsilon), -self.clip, self.clip)
        return x

    def save(self, path):
        data = {'mean':self.mean, 'var':self.var, 'count':self.count}
        with open(path, 'wb') as f:
            pickle.dump(data, f)

    def load(self, path):
        with open(path, 'rb') as f:
                data = pickle.load(f)
                self.mean = data['mean']
                self.var = data['var']
                self.count = data['count']
        
    def set_num_states(self, size):
        if size != self.mean.shape[0]:
            l = size - self.mean.shape[0]
            m_new = np.zeros(l, 'float64')
            v_new = np.ones(l, 'float64')
            self.mean = np.concatenate((self.mean, m_new), axis=0)
            self.var = np.concatenate((self.var, v_new), axis=0)
            print("new RMS state size: ", self.mean.shape)


def update_mean_var_count_from_moments(mean, var, count, batch_mean, batch_var, batch_count):
    delta = batch_mean - mean
    tot_count = count + batch_count

    new_mean = mean + delta * batch_count / tot_count
    m_a = var * count
    m_b = batch_var * batch_count
    M2 = m_a + m_b + np.square(delta) * count * batch_count / tot_count
    new_var = M2 / tot_count
    new_count = tot_count

    return new_mean, new_var, new_count
