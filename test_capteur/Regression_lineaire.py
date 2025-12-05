import numpy as np
import matplotlib.pyplot as plt

np.random.seed(3)   # pour toujours reproduire le meme dataset
n_samples = 100     # nombre d'echantillons a générer
x = np.linspace(0, 10, n_samples).reshape((n_samples, 1))
y = x + np.random.randn(n_samples, 1)


X = np.hstack((x, np.ones(x.shape)))
theta = np.random.randn(2, 1)
print((X.shape),theta)
a = []
Oprems = -2
for i in range(12):
    a.append(i)
plt.scatter(x, y)   # afficher les résultats. X en abscisse et y en ordonnée
plt.show()





