import matplotlib.pyplot as plt
import numpy as np

# 设置全局字体和样式
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['Arial']
plt.rcParams['axes.labelsize'] = 14
plt.rcParams['axes.titlesize'] = 16
plt.rcParams['xtick.labelsize'] = 12
plt.rcParams['ytick.labelsize'] = 12

# 数据
models = ['PLMA', 'PODAR', 'Driver Risk Field (DRF)', 'TransNeXt', 'RepViT']
prediction_horizon = [4.25, 4.2, np.nan, 3, np.nan]  # N/A 用 np.nan 表示

# 创建图形
fig, ax = plt.subplots(figsize=(8, 5))

# 使用 viridis 配色方案
colors = plt.cm.viridis(np.linspace(0, 1, len(models)))

# 绘制柱状图
bars = ax.bar(models, prediction_horizon, color=colors, edgecolor='black', linewidth=1.2)

# 添加数值标签
for bar in bars:
    height = bar.get_height()
    if not np.isnan(height):
        ax.text(bar.get_x() + bar.get_width() / 2., height,
                f'{height:.2f}',
                ha='center', va='bottom', fontsize=12, color='black')

# 设置标题和标签
ax.set_title('Prediction Horizon (T) Comparison', fontsize=16, pad=20)
ax.set_xlabel('Models', fontsize=14, labelpad=10)
ax.set_ylabel('Prediction Horizon (T)', fontsize=14, labelpad=10)

# 设置网格线
ax.grid(True, linestyle='--', alpha=0.6, axis='y')

# 设置 y 轴范围
ax.set_ylim(0, 5)

# 旋转 x 轴标签
plt.xticks(rotation=45, ha='right')

# 调整布局
plt.tight_layout()

# 保存为高分辨率图片
plt.savefig('prediction_horizon_comparison.pdf', format='pdf', dpi=300, bbox_inches='tight')
plt.savefig('prediction_horizon_comparison.png', format='png', dpi=300, bbox_inches='tight')

# 显示图形
plt.show()