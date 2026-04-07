import pandas as pd
import matplotlib.pyplot as plt

# ═════════════════════════════════════════════
# USER OPTIONS  ← change these as needed
# ═════════════════════════════════════════════

# Folder containing CSV files to process (use '.' for current directory)
CSV_FOLDER = '.'

# ═════════════════════════════════════════════

# ─────────────────────────────────────────────
# Discover all CSV files and process each one
# ─────────────────────────────────────────────
import os
import glob
import shutil

csv_files = sorted(glob.glob(os.path.join(CSV_FOLDER, '*.csv')))
if not csv_files:
    raise FileNotFoundError(f"No CSV files found in folder: '{CSV_FOLDER}'")

print(f"Found {len(csv_files)} CSV file(s): {[os.path.basename(f) for f in csv_files]}")

for CSV_FILE in csv_files:
    print(f"\n{'='*70}")
    print(f"Processing: {CSV_FILE}")
    print(f"{'='*70}")

    FILE_PREFIX = os.path.splitext(os.path.basename(CSV_FILE))[0] + '_'
    FOLDER_NAME = os.path.splitext(os.path.basename(CSV_FILE))[0]
    os.makedirs(FOLDER_NAME, exist_ok=True)

    df = pd.read_csv(CSV_FILE)

    # ─────────────────────────────────────────────
    # Statistical analysis helper
    # ─────────────────────────────────────────────
    # Accuracy % = (1 - MAE / mean(|reference|)) * 100
    # Clamped to [0, 100]; returns None when reference mean is zero.

    def accuracy_pct(error_series, ref_series):
        valid = error_series.dropna().index.intersection(ref_series.dropna().index)
        mean_abs_err = error_series.loc[valid].abs().mean()
        mean_abs_ref = ref_series.loc[valid].abs().mean()
        if mean_abs_ref == 0:
            return None
        return max(0.0, (1 - mean_abs_err / mean_abs_ref) * 100)

    sep  = '=' * 60
    sep2 = '-' * 60

    # ─────────────────────────────────────────────
    # Run analysis for both dynamic and static
    # ─────────────────────────────────────────────
    MODES = {
        'dynamic': ('dynamic_x', 'dynamic_y', 'dynamic_vx', 'dynamic_vy', 'Dynamic_object_1 (0x510)'),
        'static' : ('static_x',  'static_y',  'static_vx',  'static_vy',  'Static_object_1 (0x590)'),
    }

    for ANALYSIS_MODE, (OBJ_X, OBJ_Y, OBJ_VX, OBJ_VY, mode_label) in MODES.items():

        # Filter rows
        dfa = df.copy()
        total_rows = len(dfa)

        # 1. Drop rows where the object X is missing
        dfa = dfa.dropna(subset=[OBJ_X])
        after_nan_drop = len(dfa)

        # 2. Drop rows where dynamic_y is outside ±100 (outliers / invalid detections)
        dfa = dfa[dfa['dynamic_y'].isna() | dfa['dynamic_y'].between(-100, 100)]
        valid_rows = len(dfa)

        dropped_nan   = total_rows - after_nan_drop
        dropped_y_out = after_nan_drop - valid_rows
        print(f'[{ANALYSIS_MODE.upper()}] Rows total: {total_rows} | After NaN drop: {after_nan_drop} | After ±100 Y filter: {valid_rows} | Dropped (NaN): {dropped_nan} | Dropped (|dynamic_y|>100): {dropped_y_out}')

        # Error columns
        dfa['error_x']  = dfa[OBJ_X]  - dfa['HIL1_x']
        dfa['error_y']  = dfa[OBJ_Y]  - dfa['HIL1_y']
        dfa['error_vx'] = dfa[OBJ_VX] - dfa['HIL1_vx']
        dfa['error_vy'] = dfa[OBJ_VY] - dfa['HIL1_vy']

        error_meta = [
            (f'Error in X  ({OBJ_X}  - HIL1_x)',  'error_x',  'HIL1_x'),
            (f'Error in Y  ({OBJ_Y}  - HIL1_y)',  'error_y',  'HIL1_y'),
            (f'Error in Vx ({OBJ_VX} - HIL1_vx)', 'error_vx', 'HIL1_vx'),
            (f'Error in Vy ({OBJ_VY} - HIL1_vy)', 'error_vy', 'HIL1_vy'),
        ]

        # Write stats file — named with mode suffix
        STATS_FILE = os.path.join(FOLDER_NAME, FILE_PREFIX + f'statistical_analysis_{ANALYSIS_MODE}.txt')

        with open(STATS_FILE, 'w') as f:
            f.write(f'STATISTICAL ANALYSIS — {mode_label} vs HIL_Target1 (0x100)\n')
            f.write(f'{sep}\n')
            f.write(f'  Source file         : {CSV_FILE}\n')
            f.write(f'  Analysis mode       : {ANALYSIS_MODE.upper()}\n')
            f.write(f'  Total rows          : {total_rows}\n')
            f.write(f'  Dropped (NaN {OBJ_X}) : {dropped_nan}\n')
            f.write(f'  Dropped (|dynamic_y|>100): {dropped_y_out}\n')
            f.write(f'  Valid rows          : {valid_rows}\n')
            f.write(f'{sep}\n\n')

            for label, err_col, ref_col in error_meta:
                err = dfa[err_col].dropna()
                acc = accuracy_pct(dfa[err_col], dfa[ref_col])

                f.write(f'{sep}\n')
                f.write(f'  {label}\n')
                f.write(f'{sep}\n')
                f.write(f'  Valid samples      : {len(err)}\n')
                f.write(f'{sep2}\n')
                f.write(f'  Mean (avg error)   : {err.mean():>12.6f}\n')
                f.write(f'  Variance           : {err.var():>12.6f}\n')
                f.write(f'  Std Deviation      : {err.std():>12.6f}\n')
                f.write(f'{sep2}\n')
                f.write(f'  Min error          : {err.min():>12.6f}\n')
                f.write(f'  Max error          : {err.max():>12.6f}\n')
                f.write(f'  Median error       : {err.median():>12.6f}\n')
                f.write(f'  Mean Abs Error     : {err.abs().mean():>12.6f}\n')
                f.write(f'  RMS Error          : {(err**2).mean()**0.5:>12.6f}\n')
                f.write(f'{sep2}\n')
                if acc is None:
                    mae = err.abs().mean()
                    f.write(f'  Accuracy           : {"Not applicable (ground truth = 0)":>35}\n')
                    f.write(f'  (Use MAE as metric): {mae:>12.6f}\n')
                else:
                    f.write(f'  Accuracy           : {f"{acc:.2f} %":>25}\n')
                f.write(f'{sep}\n\n')

            f.write('END OF REPORT\n')

        print(f'Saved: {STATS_FILE}')


    # ─────────────────────────────────────────────
    # Column mapping  (edit here if names differ)
    # ─────────────────────────────────────────────
    # Dynamic Object 1  (CAN 0x510)
    DYN1_X  = 'dynamic_x'
    DYN1_Y  = 'dynamic_y'
    DYN1_VX = 'dynamic_vx'
    DYN1_VY = 'dynamic_vy'

    # Static Object 1  (CAN 0x590)
    STA1_X  = 'static_x'
    STA1_Y  = 'static_y'
    STA1_VX = 'static_vx'
    STA1_VY = 'static_vy'

    # HIL Target 1  (CAN 0x100)
    HIL1_X  = 'HIL1_x'
    HIL1_Y  = 'HIL1_y'
    HIL1_VX = 'HIL1_vx'
    HIL1_VY = 'HIL1_vy'


    # Time axis
    TIME = 'timestamp'


    # ─────────────────────────────────────────────
    # Helper: create a 5-panel comparison figure
    # ─────────────────────────────────────────────
    def plot_comparison(df, title,
                        obj_x, obj_y, obj_vx, obj_vy,
                        hil_x, hil_y, hil_vx, hil_vy,
                        obj_label, hil_label):
        """
        Produces one figure with 5 subplots:
          1. X-Distance vs Time
          2. Y-Distance vs Time
          3. Vx vs Time
          4. Vy vs Time
          5. Y vs X  (scatter)
        """
        fig, axes = plt.subplots(1, 5, figsize=(22, 5))
        fig.suptitle(title, fontsize=13, fontweight='bold')

        t = df[TIME]

        # ── Graph 1 : X-Distance vs Time ──────────────
        ax = axes[0]
        ax.plot(t, df[hil_x], color='tab:orange', label=hil_label, linewidth=1, linestyle='--')
        ax.plot(t, df[obj_x], color='tab:blue',   label=obj_label, linewidth=1)
        ax.set_title('X-Distance vs Time')
        ax.set_xlabel('Time')
        ax.set_ylabel('X Distance')
        ax.legend(fontsize=7)
        ax.grid(True)

        # ── Graph 2 : Y-Distance vs Time ──────────────
        ax = axes[1]
        ax.plot(t, df[hil_y], color='tab:orange', label=hil_label, linewidth=1, linestyle='--')
        ax.plot(t, df[obj_y], color='tab:blue',   label=obj_label, linewidth=1)
        ax.set_title('Y-Distance vs Time')
        ax.set_xlabel('Time')
        ax.set_ylabel('Y Distance')
        ax.legend(fontsize=7)
        ax.grid(True)

        # ── Graph 3 : Vx vs Time ──────────────────────
        ax = axes[2]
        ax.plot(t, df[hil_vx], color='tab:orange', label=hil_label, linewidth=1, linestyle='--')
        ax.plot(t, df[obj_vx], color='tab:blue',   label=obj_label, linewidth=1)
        ax.set_title('Vx vs Time')
        ax.set_xlabel('Time')
        ax.set_ylabel('Speed X')
        ax.legend(fontsize=7)
        ax.grid(True)

        # ── Graph 4 : Vy vs Time ──────────────────────
        ax = axes[3]
        ax.plot(t, df[hil_vy], color='tab:orange', label=hil_label, linewidth=1, linestyle='--')
        ax.plot(t, df[obj_vy], color='tab:blue',   label=obj_label, linewidth=1)
        ax.set_title('Vy vs Time')
        ax.set_xlabel('Time')
        ax.set_ylabel('Speed Y')
        ax.legend(fontsize=7)
        ax.grid(True)

        # ── Graph 5 : X vs Y  (scatter) ───────────────
        ax = axes[4]
        ax.scatter(df[hil_y], df[hil_x], s=6, color='tab:orange', label=hil_label, alpha=0.7, marker='x')
        ax.scatter(df[obj_y], df[obj_x], s=6, color='tab:blue',   label=obj_label, alpha=0.7)
        ax.set_title('X vs Y')
        ax.set_xlabel('Y Distance')
        ax.set_ylabel('X Distance')
        ax.legend(fontsize=7)
        ax.grid(True)

        plt.tight_layout()
        return fig


    # ─────────────────────────────────────────────
    # ── CAN comparison figures ──────────────────
    # ─────────────────────────────────────────────

    # 2-A  Dynamic_object_1  (0x510)  vs  HIL_Target1_Data  (0x100)
    fig_dyn1 = plot_comparison(
        df,
        title      = 'Dynamic_object_1 (0x510)  vs  HIL_Target1_Data (0x100)',
        obj_x=DYN1_X, obj_y=DYN1_Y, obj_vx=DYN1_VX, obj_vy=DYN1_VY,
        hil_x=HIL1_X, hil_y=HIL1_Y, hil_vx=HIL1_VX, hil_vy=HIL1_VY,
        obj_label='Dynamic_Obj_1', hil_label='HIL_Target1',
    )
    fig_dyn1.savefig(os.path.join(FOLDER_NAME, FILE_PREFIX + 'plot_dynamic_obj1_vs_hil_target1.png'), dpi=150, bbox_inches='tight')
    print('Saved:', os.path.join(FOLDER_NAME, FILE_PREFIX + 'plot_dynamic_obj1_vs_hil_target1.png'))

    # 2-B  Static_object_1  (0x590)  vs  HIL_Target1_Data  (0x100)
    fig_sta1 = plot_comparison(
        df,
        title      = 'Static_object_1 (0x590)  vs  HIL_Target1_Data (0x100)',
        obj_x=STA1_X, obj_y=STA1_Y, obj_vx=STA1_VX, obj_vy=STA1_VY,
        hil_x=HIL1_X, hil_y=HIL1_Y, hil_vx=HIL1_VX, hil_vy=HIL1_VY,
        obj_label='Static_Obj_1', hil_label='HIL_Target1',
    )
    fig_sta1.savefig(os.path.join(FOLDER_NAME, FILE_PREFIX + 'plot_static_obj1_vs_hil_target1.png'), dpi=150, bbox_inches='tight')
    print('Saved:', os.path.join(FOLDER_NAME, FILE_PREFIX + 'plot_static_obj1_vs_hil_target1.png'))


    # ─────────────────────────────────────────────
    # Move input CSV into the output folder
    # ─────────────────────────────────────────────
    csv_dest = os.path.join(FOLDER_NAME, os.path.basename(CSV_FILE))
    if os.path.abspath(CSV_FILE) != os.path.abspath(csv_dest):
        shutil.move(CSV_FILE, csv_dest)
        print(f'Moved: {CSV_FILE} → {csv_dest}')

    plt.close('all')  # free memory between files

print(f"\nAll done. Processed {len(csv_files)} file(s).")
plt.show()