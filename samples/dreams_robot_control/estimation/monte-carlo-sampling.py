#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
import seaborn as sns

class MonteCarloSampling:
    def __init__(self):
        np.random.seed(42)  # For reproducibility
        self.n_samples = 10000
    
    def estimate_pi(self):
        """Estimate π using Monte Carlo sampling."""
        x = np.random.uniform(-1, 1, self.n_samples)
        y = np.random.uniform(-1, 1, self.n_samples)
        
        inside_circle = np.sum(x**2 + y**2 <= 1)
        pi_estimate = 4 * inside_circle / self.n_samples
        
        plt.figure(figsize=(10, 10))
        plt.scatter(x[x**2 + y**2 > 1], y[x**2 + y**2 > 1], 
                   c='red', alpha=0.6, label='Outside')
        plt.scatter(x[x**2 + y**2 <= 1], y[x**2 + y**2 <= 1], 
                   c='blue', alpha=0.6, label='Inside')
        circle = plt.Circle((0, 0), 1, fill=False, color='black')
        plt.gca().add_artist(circle)
        plt.axis('equal')
        
        # Add descriptive text with formula
        description = (
            f"Monte Carlo Estimation of π\n"
            f"Estimate: {pi_estimate:.6f}\n\n"
            r"$\pi \approx 4 \cdot \frac{N_{inside}}{N_{total}}$" "\n\n"
            "Method: Generate random points in [-1,1]×[-1,1] square\n"
            "and count fraction inside unit circle."
        )
        plt.title(description, pad=20)
        plt.legend()
        plt.grid(True)
        plt.show()
        
        return pi_estimate

    def importance_sampling_example(self):
        """Demonstrate importance sampling for a heavy-tailed distribution."""
        def target_dist(x):
            return 1 / (1 + x**2)
        
        proposal_samples = stats.t.rvs(df=2, size=self.n_samples)
        proposal_pdf = stats.t.pdf(proposal_samples, df=2)
        weights = target_dist(proposal_samples) / proposal_pdf
        estimate = np.mean(weights * proposal_samples)
        
        plt.figure(figsize=(12, 6))
        x = np.linspace(-5, 5, 1000)
        plt.plot(x, target_dist(x), 'b-', label='Target (Cauchy)')
        plt.plot(x, stats.t.pdf(x, df=2), 'r--', label='Proposal (Student t)')
        plt.hist(proposal_samples, bins=50, density=True, alpha=0.3, 
                weights=weights/np.sum(weights), label='Weighted Samples')
        
        # Add descriptive text with formula
        description = (
            "Importance Sampling: Heavy-tailed Distribution\n\n"
            r"$E[f(x)] \approx \frac{1}{N}\sum_{i=1}^N f(x_i)\frac{p(x_i)}{q(x_i)}$" "\n\n"
            r"Target: Cauchy $p(x) = \frac{1}{1+x^2}$" "\n"
            r"Proposal: Student's t with df=2"
        )
        plt.title(description, pad=20)
        plt.xlabel('x')
        plt.ylabel('Density')
        plt.legend()
        plt.grid(True)
        plt.show()
        
        return estimate

    def rejection_sampling_example(self):
        """Demonstrate rejection sampling for a mixture of Gaussians."""
        def target_dist(x):
            return 0.4 * stats.norm.pdf(x, -2, 0.5) + 0.6 * stats.norm.pdf(x, 2, 1.0)
        
        x = np.random.uniform(-5, 5, self.n_samples)
        u = np.random.uniform(0, 0.5, self.n_samples)
        accepted = u <= target_dist(x)
        accepted_samples = x[accepted]
        
        plt.figure(figsize=(12, 6))
        x_plot = np.linspace(-5, 5, 1000)
        plt.plot(x_plot, target_dist(x_plot), 'b-', label='Target Distribution')
        plt.hist(accepted_samples, bins=50, density=True, alpha=0.5, 
                label='Accepted Samples')
        
        # Add descriptive text with formula
        description = (
            "Rejection Sampling: Mixture of Gaussians\n\n"
            r"$p(x) = 0.4\mathcal{N}(x|-2,0.5) + 0.6\mathcal{N}(x|2,1.0)$" "\n\n"
            r"Accept if: $u \leq \frac{p(x)}{Mq(x)}$" "\n"
            f"Acceptance rate: {len(accepted_samples)/self.n_samples:.2%}"
        )
        plt.title(description, pad=20)
        plt.xlabel('x')
        plt.ylabel('Density')
        plt.legend()
        plt.grid(True)
        plt.show()
        
        return accepted_samples

    def mcmc_sampling_example(self):
        """Demonstrate MCMC sampling using Metropolis-Hastings."""
        def target_dist(x):
            return np.exp(-0.5 * ((x + 2)**2)) + np.exp(-0.5 * ((x - 2)**2))
        
        current = 0.0
        samples = np.zeros(self.n_samples)
        acceptance_rate = 0
        
        for i in range(self.n_samples):
            proposal = current + np.random.normal(0, 1)
            ratio = target_dist(proposal) / target_dist(current)
            if np.random.random() < ratio:
                current = proposal
                acceptance_rate += 1
            samples[i] = current
        
        acceptance_rate /= self.n_samples
        
        plt.figure(figsize=(12, 6))
        x_plot = np.linspace(-6, 6, 1000)
        plt.plot(x_plot, target_dist(x_plot), 'b-', label='Target Distribution')
        plt.hist(samples, bins=50, density=True, alpha=0.5, label='MCMC Samples')
        
        # Add descriptive text with formula
        description = (
            "MCMC Sampling: Metropolis-Hastings Algorithm\n\n"
            r"$p(x) \propto e^{-\frac{1}{2}(x+2)^2} + e^{-\frac{1}{2}(x-2)^2}$" "\n\n"
            r"Accept with prob: $\alpha = \min(1, \frac{p(x_{new})}{p(x_{current})})$" "\n"
            f"Acceptance rate: {acceptance_rate:.2f}"
        )
        plt.title(description, pad=20)
        plt.xlabel('x')
        plt.ylabel('Density')
        plt.legend()
        plt.grid(True)
        plt.show()
        
        return samples, acceptance_rate

def main():
    mc = MonteCarloSampling()
    
    # Demonstrate π estimation
    print("\nEstimating π...")
    pi_estimate = mc.estimate_pi()
    print(f"π estimate: {pi_estimate:.6f}")
    
    # Demonstrate importance sampling
    print("\nDemonstrating importance sampling...")
    importance_estimate = mc.importance_sampling_example()
    print(f"Importance sampling estimate: {importance_estimate:.6f}")
    
    # Demonstrate rejection sampling
    print("\nDemonstrating rejection sampling...")
    rejection_samples = mc.rejection_sampling_example()
    print(f"Number of accepted samples: {len(rejection_samples)}")
    
    # Demonstrate MCMC sampling
    print("\nDemonstrating MCMC sampling...")
    mcmc_samples, acceptance_rate = mc.mcmc_sampling_example()
    print(f"MCMC acceptance rate: {acceptance_rate:.2f}")

if __name__ == '__main__':
    main()
