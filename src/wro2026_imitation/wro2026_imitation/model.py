"""Small neural network in plain numpy: predict, train with Adam, save and load."""
import json

import numpy as np


class MLP:
    """Fully connected network: ReLU hidden layers, one softmax output per class."""

    def __init__(self, n_in, hidden=(32, 16), n_out=3, seed=0):
        rng = np.random.default_rng(seed)
        sizes = [n_in, *hidden, n_out]
        self.w, self.b = [], []
        for i, (a, c) in enumerate(zip(sizes, sizes[1:])):
            std = np.sqrt(2.0 / a) if i < len(sizes) - 2 else 0.1 / np.sqrt(a)
            self.w.append(rng.normal(0.0, std, (a, c)))
            self.b.append(np.zeros(c))

    def _forward(self, x):
        layers = [x]
        h = x
        for w, b in zip(self.w[:-1], self.b[:-1]):
            h = np.maximum(h @ w + b, 0.0)
            layers.append(h)
        return h @ self.w[-1] + self.b[-1], layers

    @staticmethod
    def _softmax(logits):
        e = np.exp(logits - logits.max(axis=1, keepdims=True))
        return e / e.sum(axis=1, keepdims=True)

    def predict_proba(self, x):
        """Return the probability of each class for each row of x."""
        return self._softmax(self._forward(np.atleast_2d(x))[0])

    def predict(self, x):
        """Return the most likely class for each row of x."""
        return self.predict_proba(x).argmax(axis=1)

    def loss_and_grads(self, x, y, sample_weight=None):
        """Return the weighted cross-entropy and its gradients (backpropagation)."""
        n = len(x)
        s = np.ones(n) if sample_weight is None else sample_weight
        logits, layers = self._forward(x)
        probs = self._softmax(logits)
        rows = np.arange(n)
        loss = -(s * np.log(probs[rows, y] + 1e-12)).sum() / s.sum()
        d = probs.copy()
        d[rows, y] -= 1.0
        d = d * s[:, None] / s.sum()
        gw, gb = [None] * len(self.w), [None] * len(self.w)
        for i in reversed(range(len(self.w))):
            gw[i] = layers[i].T @ d
            gb[i] = d.sum(axis=0)
            if i > 0:
                d = (d @ self.w[i].T) * (layers[i] > 0.0)   # ReLU passes only active units
        return loss, gw, gb

    def n_parameters(self):
        """Return the number of trainable numbers in the network."""
        return sum(a.size for a in self.w + self.b)

    def save(self, path, **meta):
        """Write the weights and a JSON description to an .npz file."""
        arrays = {'w%d' % i: a for i, a in enumerate(self.w)}
        arrays.update({'b%d' % i: a for i, a in enumerate(self.b)})
        np.savez(path, n_layers=len(self.w), meta=json.dumps(meta), **arrays)

    @classmethod
    def load(cls, path):
        """Return (model, meta) from a file written by save()."""
        with np.load(path) as d:
            n = int(d['n_layers'])
            model = cls.__new__(cls)
            model.w = [d['w%d' % i] for i in range(n)]
            model.b = [d['b%d' % i] for i in range(n)]
            return model, json.loads(str(d['meta']))


def train(model, x, y, x_val, y_val, epochs=300, batch=64, lr=2e-3, weight_decay=1e-4,
          sample_weight=None, val_weight=None, patience=60, seed=0):
    """
    Train with mini-batch Adam and keep the weights with the lowest validation loss.

    Return (best epoch, best validation loss). `patience` epochs without
    improvement stop the training.
    """
    rng = np.random.default_rng(seed)
    params = model.w + model.b
    m1 = [np.zeros_like(p) for p in params]
    m2 = [np.zeros_like(p) for p in params]
    s = np.ones(len(x)) if sample_weight is None else sample_weight
    best_loss, best_epoch, best = np.inf, -1, None
    step = 0
    for epoch in range(epochs):
        order = rng.permutation(len(x))
        for start in range(0, len(x), batch):
            pick = order[start:start + batch]
            _, gw, gb = model.loss_and_grads(x[pick], y[pick], s[pick])
            grads = [g + weight_decay * w for g, w in zip(gw, model.w)] + gb
            step += 1
            for i, (p, g) in enumerate(zip(params, grads)):
                m1[i] = 0.9 * m1[i] + 0.1 * g
                m2[i] = 0.999 * m2[i] + 0.001 * g * g
                p -= lr * (m1[i] / (1 - 0.9 ** step)) \
                    / (np.sqrt(m2[i] / (1 - 0.999 ** step)) + 1e-8)
        loss = model.loss_and_grads(x_val, y_val, val_weight)[0]
        if loss < best_loss - 1e-4:
            best_loss, best_epoch = loss, epoch
            best = [p.copy() for p in params]
        elif epoch - best_epoch >= patience:
            break
    for p, saved in zip(params, best):
        p[...] = saved
    return best_epoch, best_loss


def confusion(y_true, y_pred, n=3):
    """Return the n x n table: rows are the true class, columns the predicted class."""
    table = np.zeros((n, n), dtype=int)
    for t, p in zip(y_true, y_pred):
        table[t, p] += 1
    return table


def balanced_accuracy(table):
    """Return the mean recall of the classes present in a confusion table."""
    seen = table.sum(axis=1) > 0
    return float((table.diagonal()[seen] / table.sum(axis=1)[seen]).mean())
