#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm, multivariate_normal
import seaborn as sns

class PhoenixHeightSampling:
    def __init__(self, n_samples=10000):
        self.n_samples = n_samples
        np.random.seed(42)  # For reproducibility
        
        # Define city centers (x, y) in miles relative to downtown Phoenix
        self.city_centers = {
            'Phoenix': np.array([0.0, 0.0]),
            'Mesa': np.array([20.0, 0.0]),     # Approximately 20 miles east
            'Tempe': np.array([10.0, -5.0])    # Approximately 10 miles east, 5 miles south
        }
        
        # Population density parameters for each city (make them more equal for trimodal)
        self.population_weights = {
            'Phoenix': 0.35,    # 35% of population
            'Mesa': 0.35,       # 35% of population
            'Tempe': 0.30       # 30% of population
        }
        
        # Spatial spread of each city (in miles) - make them smaller for distinct modes
        self.city_spreads = {
            'Phoenix': 5.0,    # Smaller spread for more distinct modes
            'Mesa': 4.0,
            'Tempe': 3.0
        }
        
        # Height distribution parameters for each city (in inches)
        self.height_params = {
            'Phoenix': {'mean': 70.0, 'std': 2.5},  # Much taller population
            'Mesa': {'mean': 66.0, 'std': 3.0},     # Average height
            'Tempe': {'mean': 63.0, 'std': 2.0}     # College town, shorter and more uniform
        }
        
        # Initialize samples and weights
        self.samples = None
        self.weights = None
        self.sample_cities = None  # Track which city each sample belongs to
        
    def generate_proposal_distribution(self):
        """
        Generate samples from a proposal distribution (uniform over the region)
        Returns: Array of [x, y] coordinates
        """
        # Sample uniformly over a rectangular region covering all cities
        x_min, x_max = -15, 35
        y_min, y_max = -15, 15
        
        x = np.random.uniform(x_min, x_max, self.n_samples)
        y = np.random.uniform(y_min, y_max, self.n_samples)
        
        return np.column_stack((x, y))
    
    def target_density(self, locations):
        """
        Calculate target density (population density) at given locations
        """
        density = np.zeros(len(locations))
        
        for city, center in self.city_centers.items():
            # Calculate multivariate normal density for each city
            spread = self.city_spreads[city]
            weight = self.population_weights[city]
            
            city_density = multivariate_normal.pdf(
                locations, 
                mean=center, 
                cov=np.eye(2) * spread**2
            )
            density += weight * city_density
        
        return density
    
    def sample_heights(self, locations):
        """
        Sample heights based on location-dependent distribution
        """
        heights = np.zeros(len(locations))
        self.sample_cities = np.zeros(len(locations), dtype=str)
        
        # Calculate distances to each city center
        distances = {}
        for city, center in self.city_centers.items():
            distances[city] = np.linalg.norm(locations - center, axis=1)
        
        # Assign each point to nearest city and sample from that city's distribution
        for i in range(len(locations)):
            # Find nearest city
            nearest_city = min(distances.keys(), key=lambda c: distances[c][i])
            self.sample_cities[i] = nearest_city
            
            # Sample height from that city's distribution
            city_params = self.height_params[nearest_city]
            heights[i] = np.random.normal(city_params['mean'], city_params['std'])
            
            # Add small elevation effect
            heights[i] += 0.05 * locations[i, 1]
        
        return heights
    
    def importance_sampling(self):
        """
        Perform importance sampling to estimate height distribution
        """
        # Generate samples from proposal distribution
        locations = self.generate_proposal_distribution()
        
        # Calculate importance weights
        target_densities = self.target_density(locations)
        proposal_density = 1.0 / ((35 - (-15)) * (15 - (-15)))  # Uniform proposal
        self.weights = target_densities / proposal_density
        self.weights /= np.sum(self.weights)  # Normalize weights
        
        # Sample heights
        self.samples = self.sample_heights(locations)
        
        return locations, self.samples, self.weights
    
    def visualize(self):
        """
        Visualize population density and height distribution
        """
        # Create figure with subplots
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # Plot 1: Population density and samples
        x = np.linspace(-15, 35, 100)
        y = np.linspace(-15, 15, 100)
        X, Y = np.meshgrid(x, y)
        pos = np.dstack((X, Y))
        
        # Calculate density for visualization
        Z = self.target_density(pos.reshape(-1, 2)).reshape(X.shape)
        
        # Plot density map
        density_plot = ax1.contourf(X, Y, Z, levels=20, cmap='viridis')
        plt.colorbar(density_plot, ax=ax1, label='Population Density')
        
        # Define city colors
        city_colors = {
            'Phoenix': 'red',
            'Mesa': 'green',
            'Tempe': 'blue'
        }
        
        # Plot city centers with different colors
        for city, center in self.city_centers.items():
            ax1.plot(center[0], center[1], '*', color=city_colors[city], 
                    markersize=15, label=f'{city}', markeredgecolor='white')
        
        ax1.set_title('Population Density Map\n(Trimodal Distribution)')
        ax1.set_xlabel('Miles East of Downtown Phoenix')
        ax1.set_ylabel('Miles North of Downtown Phoenix')
        ax1.legend()
        ax1.grid(True)
        
        # Plot 2: Height distribution
        ax2.hist(self.samples, weights=self.weights, bins=40, density=True,
                alpha=0.7, label='Estimated Distribution')
        
        # Add theoretical distributions for each city using same colors
        x = np.linspace(60, 75, 200)
        for city, params in self.height_params.items():
            ax2.plot(x, norm.pdf(x, params['mean'], params['std']) * self.population_weights[city],
                    '--', color=city_colors[city], label=f'{city} Distribution')
        
        ax2.set_title('Height Distribution by City')
        ax2.set_xlabel('Height (inches)')
        ax2.set_ylabel('Density')
        ax2.legend()
        
        plt.tight_layout()
        plt.show()
        
        # Additional plot: Height distribution by city
        plt.figure(figsize=(10, 6))
        for city in self.city_centers.keys():
            city_mask = self.sample_cities == city
            if np.any(city_mask):
                plt.hist(self.samples[city_mask], weights=self.weights[city_mask],
                        bins=30, density=True, alpha=0.5, color=city_colors[city], label=city)
        plt.title('Height Distribution by City (Separated)')
        plt.xlabel('Height (inches)')
        plt.ylabel('Density')
        plt.legend()
        plt.grid(True)
        plt.show()

def main():
    # Create sampler
    sampler = PhoenixHeightSampling(n_samples=1000000)
    
    # Perform importance sampling
    locations, heights, weights = sampler.importance_sampling()
    
    # Calculate weighted statistics
    weighted_mean = np.average(heights, weights=weights)
    weighted_std = np.sqrt(np.average((heights - weighted_mean)**2, weights=weights))
    
    print(f"Estimated population statistics:")
    print(f"Mean height: {weighted_mean:.2f} inches")
    print(f"Standard deviation: {weighted_std:.2f} inches")
    
    # Visualize results
    sampler.visualize()

if __name__ == '__main__':
    main() 