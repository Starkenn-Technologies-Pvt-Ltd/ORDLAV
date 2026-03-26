import pandas as pd
import matplotlib.pyplot as plt

# ═════════════════════════════════════════════
# USER OPTIONS  ← change these as needed
# ═════════════════════════════════════════════

CSV_FILE = 'Truck_MT_10kmph_parsed.csv'

# Choose which object to analyse:
#   'dynamic' → Dynamic_object_1 (CAN 0x510) vs HIL_Target1 (CAN 0x100)
#   'static'  → Static_object_1  (CAN 0x590) vs HIL_Target1 (CAN 0x100)
ANALYSIS_MODE = 'dynamic'   # ← set to 'dynamic' or 'static'

# ═════════════════════════════════════════════

# ─────────────────────────────────────────────
# Load data
# ─────────────────────────────────────────────
# Derive prefix from input filename — e.g. "my_data.csv" → "my_data_"
import os
FILE_PREFIX = os.path.splitext(os.path.basename(CSV_FILE))[0] + '_'

df = pd.read_csv(CSV_FILE)

# ─────────────────────────────────────────────
# Select object columns based on ANALYSIS_MODE
# ─────────────────────────────────────────────
if ANALYSIS_MODE == 'dynamic':
    OBJ_X, OBJ_Y, OBJ_VX, OBJ_VY = 'dynamic_x', 'dynamic_y', 'dynamic_vx', 'dynamic_vy'
    mode_label = 'Dynamic_object_1 (0x510)'
elif ANALYSIS_MODE == 'static':
    OBJ_X, OBJ_Y, OBJ_VX, OBJ_VY = 'static_x', 'static_y', 'static_vx', 'static_vy'
    mode_label = 'Static_object_1 (0x590)'
else:
    raise ValueError(f"ANALYSIS_MODE must be 'dynamic' or 'static', got: '{ANALYSIS_MODE}'")

# ─────────────────────────────────────────────
# Filter rows for analysis
# ─────────────────────────────────────────────
total_rows = len(df)

# 1. Drop rows where the object X is missing
df = df.dropna(subset=[OBJ_X])
after_nan_drop = len(df)

# 2. Drop rows where dynamic_y is outside ±100 (outliers / invalid detections)
df = df[df['dynamic_y'].isna() | df['dynamic_y'].between(-100, 100)]
valid_rows = len(df)

dropped_nan   = total_rows - after_nan_drop
dropped_y_out = after_nan_drop - valid_rows
print(f'[{ANALYSIS_MODE.upper()}] Rows total: {total_rows} | After NaN drop: {after_nan_drop} | After ±100 Y filter: {valid_rows} | Dropped (NaN): {dropped_nan} | Dropped (|dynamic_y|>100): {dropped_y_out}')

# ─────────────────────────────────────────────
# Error columns
# ─────────────────────────────────────────────
df['error_x']  = df[OBJ_X]  - df['HIL1_x']
df['error_y']  = df[OBJ_Y]  - df['HIL1_y']
df['error_vx'] = df[OBJ_VX] - df['HIL1_vx']
df['error_vy'] = df[OBJ_VY] - df['HIL1_vy']

# ─────────────────────────────────────────────
# Statistical analysis → text file
# ─────────────────────────────────────────────
# Accuracy % = (1 - MAE / mean(|reference|)) * 100
# Clamped to [0, 100]; shows N/A when reference mean is zero.

def accuracy_pct(error_series, ref_series):
    valid = error_series.dropna().index.intersection(ref_series.dropna().index)
    mean_abs_err = error_series.loc[valid].abs().mean()
    mean_abs_ref = ref_series.loc[valid].abs().mean()
    if mean_abs_ref == 0:
        # Reference ground truth is 0; % accuracy undefined (division by zero).
        # Return None so the report can show MAE-based info instead.
        return None
    return max(0.0, (1 - mean_abs_err / mean_abs_ref) * 100)

error_meta = [
    (f'Error in X  ({OBJ_X}  - HIL1_x)',  'error_x',  'HIL1_x'),
    (f'Error in Y  ({OBJ_Y}  - HIL1_y)',  'error_y',  'HIL1_y'),
    (f'Error in Vx ({OBJ_VX} - HIL1_vx)', 'error_vx', 'HIL1_vx'),
    (f'Error in Vy ({OBJ_VY} - HIL1_vy)', 'error_vy', 'HIL1_vy'),
]

STATS_FILE = FILE_PREFIX + 'statistical_analysis.txt'
sep  = '=' * 60
sep2 = '-' * 60

with open(STATS_FILE, 'w') as f:
    f.write(f'STATISTICAL ANALYSIS — {mode_label} vs HIL_Target1 (0x100)\n')
    f.write(f'{sep}\n')
    f.write(f'  Source file    : {CSV_FILE}\n')
    f.write(f'  Analysis mode  : {ANALYSIS_MODE.upper()}\n')
    f.write(f'  Total rows          : {total_rows}\n')
    f.write(f'  Dropped (NaN {OBJ_X}) : {dropped_nan}\n')
    f.write(f'  Dropped (|dynamic_y|>100): {dropped_y_out}\n')
    f.write(f'  Valid rows          : {valid_rows}\n')
    f.write(f'{sep}\n\n')

    for label, err_col, ref_col in error_meta:
        err = df[err_col].dropna()
        acc = accuracy_pct(df[err_col], df[ref_col])

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

# HIL Target 2  (CAN 0x101)
HIL2_X  = 'HIL2_x'
HIL2_Y  = 'HIL2_y'
HIL2_VX = 'HIL2_vx'
HIL2_VY = 'HIL2_vy'

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
    ax.plot(t, df[obj_vx], color='tab:blue',   label=obj_label, linewidth=1)
    ax.plot(t, df[hil_vx], color='tab:orange', label=hil_label, linewidth=1, linestyle='--')
    ax.set_title('Vx vs Time')
    ax.set_xlabel('Time')
    ax.set_ylabel('Speed X')
    ax.legend(fontsize=7)
    ax.grid(True)

    # ── Graph 4 : Vy vs Time ──────────────────────
    ax = axes[3]
    ax.plot(t, df[obj_vy], color='tab:blue',   label=obj_label, linewidth=1)
    ax.plot(t, df[hil_vy], color='tab:orange', label=hil_label, linewidth=1, linestyle='--')
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
# ── SECTION 1 : Original scatter plots ───────
# ─────────────────────────────────────────────
moving_towards     = df[df[DYN1_VX] < 0]
moving_away        = df[df[DYN1_VX] > 0]
moving_toward_hil  = df[df[HIL1_VX] < 0]
moving_away_hil    = df[df[HIL1_VX] > 0]

fig0, (ax1, ax2, ax3, ax4) = plt.subplots(1, 4, figsize=(14, 6), sharey=True)
fig0.suptitle('Scatter Plot: Moving Toward vs Moving Away')


ax1.scatter(moving_towards[DYN1_Y],    moving_towards[DYN1_X],   s=8, color='red')
ax1.set_title('Moving Toward');  ax1.set_xlabel('dynamic_y'); ax1.set_ylabel('dynamic_x'); ax1.grid(True)

ax2.scatter(moving_away[DYN1_Y],       moving_away[DYN1_X],      s=8, color='blue')
ax2.set_title('Moving Away');    ax2.set_xlabel('dynamic_x'); ax2.grid(True)

ax3.scatter(moving_toward_hil[HIL1_Y], moving_toward_hil[HIL1_X], s=8, color='green')
ax3.set_title('Moving Toward_HIL'); ax3.set_xlabel('dynamic_y'); ax3.grid(True)

ax4.scatter(moving_away_hil[HIL1_Y],  moving_away_hil[HIL1_X],  s=8, color='violet')
ax4.set_title('Moving Away_HIL'); ax4.set_xlabel('dynamic_y'); ax4.grid(True)

for ax in (ax1, ax2, ax3, ax4):
    ax.set_ylim(0, 60)

plt.tight_layout()


# ─────────────────────────────────────────────
# ── SECTION 2 : CAN comparison figures ───────
# ─────────────────────────────────────────────

# ── Figure 1 : Original scatter plots ────────────────────────────────────────
fig0.savefig(FILE_PREFIX + 'plot_scatter_moving_toward_away.png', dpi=150, bbox_inches='tight')
print('Saved:', FILE_PREFIX + 'plot_scatter_moving_toward_away.png')

# 2-A  Dynamic_object_1  (0x510)  vs  HIL_Target1_Data  (0x100)
fig_dyn1 = plot_comparison(
    df,
    title      = 'Dynamic_object_1 (0x510)  vs  HIL_Target1_Data (0x100)',
    obj_x=DYN1_X, obj_y=DYN1_Y, obj_vx=DYN1_VX, obj_vy=DYN1_VY,
    hil_x=HIL1_X, hil_y=HIL1_Y, hil_vx=HIL1_VX, hil_vy=HIL1_VY,
    obj_label='Dynamic_Obj_1', hil_label='HIL_Target1',
)
fig_dyn1.savefig(FILE_PREFIX + 'plot_dynamic_obj1_vs_hil_target1.png', dpi=150, bbox_inches='tight')
print('Saved:', FILE_PREFIX + 'plot_dynamic_obj1_vs_hil_target1.png')

# 2-B  Static_object_1  (0x590)  vs  HIL_Target1_Data  (0x100)
fig_sta1 = plot_comparison(
    df,
    title      = 'Static_object_1 (0x590)  vs  HIL_Target1_Data (0x100)',
    obj_x=STA1_X, obj_y=STA1_Y, obj_vx=STA1_VX, obj_vy=STA1_VY,
    hil_x=HIL1_X, hil_y=HIL1_Y, hil_vx=HIL1_VX, hil_vy=HIL1_VY,
    obj_label='Static_Obj_1', hil_label='HIL_Target1',
)
fig_sta1.savefig(FILE_PREFIX + 'plot_static_obj1_vs_hil_target1.png', dpi=150, bbox_inches='tight')
print('Saved:', FILE_PREFIX + 'plot_static_obj1_vs_hil_target1.png')

plt.show()