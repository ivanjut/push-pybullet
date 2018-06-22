import numpy as np
import sklearn.gaussian_process as gp

from scipy.stats import norm
from scipy.optimize import minimize

def expected_improvement(x, gaussian_process, evaluated_loss, maximize=True, n_params=1):
    x_to_predict = x.reshape(-1, n_params)

    mu, sigma = gaussian_process.predict(x_to_predict, return_std=True)

    if maximize:
        loss_optimum = np.max(evaluated_loss)
    else:
        loss_optimum = np.min(evaluated_loss)

    scaling_factor = (-1) ** (not maximize)

    # In case sigma equals zero
    with np.errstate(divide='ignore'):
        Z = scaling_factor * (mu - loss_optimum) / sigma
        expected_improvement = scaling_factor * (mu - loss_optimum) * norm.cdf(Z) + sigma * norm.pdf(Z)
        expected_improvement[sigma == 0.0] == 0.0

    return -1 * expected_improvement

def prob_of_improvement(x, gaussian_process, evaluated_loss, maximize=True, n_params=1):
    x_to_predict = x.reshape(-1, n_params)

    mu, sigma = gaussian_process.predict(x_to_predict, return_std=True)

    if maximize:
        loss_optimum = np.max(evaluated_loss)
    else:
        loss_optimum = np.min(evaluated_loss)

    scaling_factor = (-1) ** (not maximize)

    # In case sigma equals zero
    with np.errstate(divide='ignore'):
        Z = scaling_factor * (mu - loss_optimum) / sigma
        prob_of_improvement = norm.cdf(Z)

    return -1 * prob_of_improvement

def upper_confidence_bound(x, gaussian_process, evaluated_loss, maximize=True, n_params=1):
    x_to_predict = x.reshape(-1, n_params)

    mu, sigma = gaussian_process.predict(x_to_predict, return_std=True)

    scaling_factor = (-1) ** (not maximize)

    with np.errstate(divide='ignore'):
        upper_confidence_bound = mu - 50.0*sigma

    return -1 * upper_confidence_bound

def sample_next_hyperparameter(acquisition_func, gaussian_process, evaluated_loss, maximize=False,
                               bounds=(0, 10), n_restarts=25):
    best_x = None
    best_acquisition_value = 1
    n_params = bounds.shape[0]

    for starting_point in np.random.uniform(bounds[:, 0], bounds[:, 1], size=(n_restarts, n_params)):

        res = minimize(fun=acquisition_func,
                       x0=starting_point.reshape(1, -1),
                       bounds=bounds,
                       method='L-BFGS-B',
                       args=(gaussian_process, evaluated_loss, maximize, n_params))

        if res.fun < best_acquisition_value:
            best_acquisition_value = res.fun
            best_x = res.x

    return best_x


def bayesian_optimisation(f, bounds, maximize, iterations, x0=None, n_pre_samples=100):
    alpha, epsilon =1e-5,1e-7
    
    acquisition_func = expected_improvement

    x_list = []
    y_list = []

    n_params = bounds.shape[0]
    for params in np.random.uniform(bounds[:, 0], bounds[:, 1], (n_pre_samples, bounds.shape[0])):
        x_list.append(params)
        y_list.append(f(*params))

    xp = np.array(x_list)
    yp = np.array(y_list)

    kernel = gp.kernels.Matern()
    model = gp.GaussianProcessRegressor(kernel=kernel,
                                        alpha=alpha,
                                        n_restarts_optimizer=10,
                                        normalize_y=True)

    for n in range(iterations):
        model.fit(xp, yp)

        # Sample next hyperparameter
        next_sample = sample_next_hyperparameter(acquisition_func, model, yp, maximize=maximize, bounds=bounds, n_restarts=100)
        print next_sample
        # Duplicates will break the GP. In case of a duplicate, we will randomly sample a next query point.
        if np.any(np.abs(next_sample - xp) <= epsilon):
            next_sample = np.random.uniform(bounds[:, 0], bounds[:, 1], bounds.shape[0])

        # Sample loss for new set of parameters
        cv_score = f(*next_sample)
        print cv_score

        # Update lists
        x_list.append(next_sample)
        y_list.append(cv_score)

        # Update xp and yp
        xp = np.array(x_list)
        yp = np.array(y_list)

    return xp, yp

# Rosenbrock function: a,b = bayesian_optimisation(lambda x,y: ((1-x)**2 + 100*(y-x**2)**2),np.array([[-2,2],[-2,2]]),False, 100)
# Himmelblau's function: a,b = bayesian_optimisation(lambda x,y: ((x**2 + y -11)**2 + (x + y**2 -7)**2),np.array([[-5,5],[-5,5]]),False, 100)