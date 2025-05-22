import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D # For 3D surface plot
import numpy as np
import os
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))

# --- Function to find CSV (similar to your multi_drone_plotter) ---
def find_csv(filename):
    # Simplified: Assumes CSV is in the same dir as script, or one level up in ../build/Debug
    # You should make this robust based on where your C++ executable writes the files.
    path1 = os.path.join(script_dir, filename)
    # Path if script is in 'final' and CSV in 'final/agent_control_pkg/build/Debug'
    path2 = os.path.join(os.path.dirname(script_dir), "agent_control_pkg", "build", "Debug", filename)
    
    if os.path.exists(path1):
        print(f"Found {filename} at: {path1}")
        return path1
    elif os.path.exists(path2):
        print(f"Found {filename} at: {path2}")
        return path2
    else:
        print(f"Error: {filename} not found at expected locations.")
        print(f"Searched: {path1}")
        print(f"Searched: {path2}")
        return None

# --- 1. Plot Membership Functions ---
mf_csv_path = find_csv("fls_mf_data.csv")
if mf_csv_path:
    try:
        df_mf = pd.read_csv(mf_csv_path)
        
        input_vars = df_mf['InputVar'].unique()
        num_vars = len(input_vars)
        
        fig_mf, axs_mf = plt.subplots(num_vars, 1, figsize=(10, num_vars * 4))
        if num_vars == 1: # If only one var, axs_mf is not an array
            axs_mf = [axs_mf] 
        fig_mf.suptitle('FLS Input Membership Functions (IT2-FLS FOUs)', fontsize=16)

        for i, var_name in enumerate(input_vars):
            ax = axs_mf[i]
            var_data = df_mf[df_mf['InputVar'] == var_name]
            set_names = var_data['SetName'].unique()
            for set_name in set_names:
                set_data = var_data[var_data['SetName'] == set_name]
                ax.plot(set_data['CrispInput'], set_data['UpperMu'], label=f'{set_name} (UMF)')
                ax.plot(set_data['CrispInput'], set_data['LowerMu'], label=f'{set_name} (LMF)', linestyle='--')
                # Fill between UMF and LMF to show FOU
                ax.fill_between(set_data['CrispInput'], set_data['LowerMu'], set_data['UpperMu'], alpha=0.2)

            ax.set_title(f'Membership Functions for "{var_name}"')
            ax.set_xlabel('Crisp Input Value')
            ax.set_ylabel('Membership Degree')
            ax.legend(fontsize='small', loc='upper right')
            ax.grid(True)
            ax.set_ylim([-0.05, 1.05])

        fig_mf.tight_layout(rect=[0, 0.03, 1, 0.95])
        fig_mf.savefig("fls_membership_functions.png", dpi=150)
        print("Saved FLS Membership Functions plot to fls_membership_functions.png")

    except Exception as e:
        print(f"Error plotting membership functions: {e}")
else:
    print("Skipping MF plot as fls_mf_data.csv was not found.")


# --- 2. Plot Control Surface ---
surface_csv_path = find_csv("fls_surface_data.csv")
if surface_csv_path:
    try:
        df_surface = pd.read_csv(surface_csv_path)

        if not df_surface.empty and all(col in df_surface.columns for col in ['Error', 'Wind', 'Correction']):
            fig_surf = plt.figure(figsize=(12, 9))
            ax_surf = fig_surf.add_subplot(111, projection='3d')

            # Create grid data for the surface plot
            X = df_surface['Error'].unique()
            Y = df_surface['Wind'].unique()
            X.sort()
            Y.sort()
            X_grid, Y_grid = np.meshgrid(X, Y)
            
            # Pivot table to get Z values in the correct shape for surf plot
            # Ensure an index is set for pivoting if duplicates exist, or aggregate
            # For simplicity, if duplicates, this might take the mean by default
            Z_grid_df = df_surface.pivot_table(index='Wind', columns='Error', values='Correction')
            Z_grid = Z_grid_df.reindex(index=Y, columns=X).values # Ensure order matches meshgrid


            if X_grid.shape == Z_grid.shape:
                surf = ax_surf.plot_surface(X_grid, Y_grid, Z_grid, cmap='viridis', edgecolor='none')
                ax_surf.set_xlabel('Error')
                ax_surf.set_ylabel('Wind')
                ax_surf.set_zlabel('FLS Correction Output')
                fixed_dError_val = df_surface['dError_fixed'].iloc[0] if 'dError_fixed' in df_surface.columns else "N/A"
                ax_surf.set_title(f'FLS Control Surface (dError fixed at {fixed_dError_val})')
                fig_surf.colorbar(surf, shrink=0.5, aspect=5)
                fig_surf.tight_layout()
                fig_surf.savefig("fls_control_surface.png", dpi=150)
                print("Saved FLS Control Surface plot to fls_control_surface.png")
            else:
                print("Error: Shape mismatch between X_grid/Y_grid and Z_grid for surface plot.")
                print(f"X_grid shape: {X_grid.shape}, Y_grid shape: {Y_grid.shape}, Z_grid shape: {Z_grid.shape}")

        else:
            print("Control surface data is empty or missing required columns (Error, Wind, Correction).")

    except Exception as e:
        print(f"Error plotting control surface: {e}")
else:
    print("Skipping Control Surface plot as fls_surface_data.csv was not found.")

plt.show() # Show all generated figures