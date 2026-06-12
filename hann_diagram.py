import numpy as np
import matplotlib.pyplot as plt

# 1. Define the parameters
N = 100  # Window length (must be even for 50% overlap)
hop = N // 2
total_length = N + hop

# Define the discrete sample indices
n = np.arange(N)

# Calculate the Hann window using the formula from your report
# Note: For strict COLA, periodic windows (denominator N) are often used, 
# but we will use the standard symmetric definition here.
w = 0.5 * (1 - np.cos(2 * np.pi * n / N))

# 2. Set up the padded arrays to position them in time
window_1 = np.zeros(total_length)
window_2 = np.zeros(total_length)

# Place Window 1 starting at index 0
window_1[0:N] = w

# Place Window 2 starting at the hop index (50% overlap)
window_2[hop:hop+N] = w

# Calculate the sum of the overlapping region
# We only care about the region where both windows exist (from 'hop' to 'N')
overlap_sum = window_1[hop:N] + window_2[hop:N]

# 3. Create the plot
# Use a wider aspect ratio (e.g., 8x3.5) which looks better in academic papers
fig, ax = plt.subplots(figsize=(8, 3.5))

# Plot the individual windows
ax.plot(window_1, label='Window $x_1$', color='#1f77b4', linewidth=2)
ax.plot(window_2, label='Window $x_2$', color='#ff7f0e', linewidth=2)

# Plot the sum in the overlap region
ax.plot(range(hop, N), overlap_sum, color='black', linestyle='--', 
        linewidth=2.5, label='Overlap Sum ($w_1 + w_2$)')

# Highlight the overlap region with a light background
ax.axvspan(hop, N-1, color='gray', alpha=0.1, label='Overlap Region')

# 4. Formatting for a professional look
ax.set_title('Constant Overlap-Add (COLA) Property of Hann Windows', fontsize=12)
ax.set_xlabel('Sample Index ($n$)', fontsize=11)
ax.set_ylabel('Window Weight', fontsize=11)

# Set axis limits
ax.set_xlim(0, total_length - 1)
ax.set_ylim(0, 1.1)

# Clean up ticks
ax.set_xticks([0, hop, N, total_length])
ax.set_xticklabels(['0', '$N/2$', '$N$', '$N + N/2$'])

# Add grid and legend
ax.grid(True, linestyle=':', alpha=0.6)
ax.legend(loc='lower center', bbox_to_anchor=(0.5, -0.35), ncol=4, frameon=False)

# Adjust layout to prevent clipping of the legend
plt.tight_layout()

# Save as a vector graphic (PDF) for LaTeX
plt.savefig('hann_blending.pdf', bbox_inches='tight')

print("Diagram saved successfully as 'hann_blending.pdf'.")