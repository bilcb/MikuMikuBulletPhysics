"""
coord.py — 坐标和单位转换的统一出口。

Blender 坐标系: Z-up, 右手系
MMD/PMX 坐标系: Y-up, 右手系
转换方式: Y/Z 轴交换

所有 Blender↔PMX 的坐标转换都应通过此模块，不要内联写。
"""


def blender_to_pmx_vec(vec, scale=1.0):
    """Blender 位置 → PMX 位置 (Y/Z swap + 缩放)"""
    return (vec[0] * scale, vec[2] * scale, vec[1] * scale)


def blender_to_pmx_euler(euler):
    """Blender 欧拉角 (YXZ) → PMX 欧拉角 (取反 + Y/Z swap)"""
    return (-euler[0], -euler[2], -euler[1])


def blender_to_pmx_size(size):
    """Blender 尺寸 → PMX 尺寸 (Y/Z swap)"""
    return (size[0], size[2], size[1])


def blender_to_pmx_axis(axis):
    """Blender 轴向量 → PMX 轴向量 (Y/Z swap)"""
    return (axis[0], axis[2], axis[1])


def blender_bone_to_mmd_conv_matrix(blender_bone_mat3):
    """将 Blender bone 的 3x3 旋转矩阵转换为 MMD 空间的变换矩阵。

    Blender bone.matrix_local 的 3x3 旋转矩阵 M 表示骨骼局部轴在 Blender 世界中的方向。
    MMD 使用 Y-up，Blender 使用 Z-up，差异在于 Y/Z swap (矩阵 S)。
    骨骼在 MMD 空间的旋转 = S * M * S。
    从 MMD 空间到骨骼局部空间的变换 = (S * M)^{-1} = M^T * S（正交矩阵）。

    Args:
        blender_bone_mat3: 3x3 矩阵，9 个元素的扁平序列（行主序）
    Returns:
        9 个元素的扁平元组（行主序），表示从 MMD 空间到骨骼局部空间的变换矩阵
    """
    m = blender_bone_mat3
    # Y/Z swap 行，然后转置
    # 原始: row0=m[0:3], row1=m[3:6], row2=m[6:9]
    # swap: row0=m[0:3], row1=m[6:9], row2=m[3:6]
    # 转置: col_i = swap_row_i
    return (
        m[0], m[6], m[3],
        m[1], m[7], m[4],
        m[2], m[8], m[5],
    )
