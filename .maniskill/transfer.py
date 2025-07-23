import open3d as o3d
import numpy as np
import time

# def stl_to_convex_hull(input_file, output_file):
#     # 读取STL文件
#     mesh = o3d.io.read_triangle_mesh(input_file)
    
#     # 获取顶点
#     vertices = np.asarray(mesh.vertices)
    
#     # 计算凸包
#     hull = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(vertices))
#     hull.estimate_normals()
#     convex_hull, _ = hull.compute_convex_hull()
    
#     # 计算法线
#     convex_hull.compute_vertex_normals()
    
#     # 保存凸包STL
#     o3d.io.write_triangle_mesh(output_file, convex_hull)
#     print(f"凸包已保存到: {output_file}")
def compare_methods(input_file, output_file):
    """
    对比两种方法的性能
    """
    print(f"性能对比测试: {input_file}")
    
    # 方法1: 您的原始方法
    start_time = time.time()
    mesh1 = o3d.io.read_triangle_mesh(input_file)
    vertices = np.asarray(mesh1.vertices)
    hull = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(vertices))
    hull.estimate_normals()
    convex_hull1, _ = hull.compute_convex_hull()
    method1_time = time.time() - start_time
    
    # 方法2: 直接方法
    start_time = time.time()
    mesh2 = o3d.io.read_triangle_mesh(input_file)
    convex_hull2, _ = mesh2.compute_convex_hull()
    method2_time = time.time() - start_time
    
    print(f"方法1 (点云中转): {method1_time:.4f} 秒")
    print(f"方法2 (直接计算): {method2_time:.4f} 秒")
    print(f"性能提升: {((method1_time - method2_time) / method1_time * 100):.1f}%")
    
    # 验证结果是否相同
    vertices1 = len(convex_hull1.vertices)
    vertices2 = len(convex_hull2.vertices)
    faces1 = len(convex_hull1.triangles)
    faces2 = len(convex_hull2.triangles)
    
    print(f"结果对比:")
    print(f"方法1: {vertices1} 顶点, {faces1} 面")
    print(f"方法2: {vertices2} 顶点, {faces2} 面")
    print(f"结果{'相同' if vertices1 == vertices2 and faces1 == faces2 else '不同'}")

    # 保存结果（使用更快的方法2）
    convex_hull2.compute_vertex_normals()
    success = o3d.io.write_triangle_mesh(output_file, convex_hull2)
    
    if success:
        print(f"✅ 凸包已保存到: {output_file}")
    else:
        print(f"❌ 保存失败: {output_file}")
    
    return convex_hull2, method2_time


# 使用示例
# stl_files = [
#     "base_link.stl",
#     "camera_mount_d405.stl",
#     "carriage_right.stl",
#     "gripper_left.stl",
#     "link_1.stl",
#     "link_2.stl",
#     "link_3.stl",
#     "link_4.stl",
#     "link_5.stl",
#     "link_6.stl",
#     "carriage_left.stl",
#     "gripper_right.stl",
#     # 添加更多 STL 文件名
# ]
stl_files = [
    'link_base.obj',
    'link1.obj',
    'link2.obj',
    'link3.obj',
    'link4.obj',
    'link5.obj',
    'link6.obj',
    'link7.obj',
]

# 批量转换
for stl_file in stl_files:
    output_file = stl_file.replace('.obj', '.obj.convex.stl')
    compare_methods("/home/pc/ManiSkill/mani_skill/assets/robots/xarm7/meshes/collision/"+stl_file, "/home/pc/ManiSkill/mani_skill/assets/robots/xarm7/meshes/collision/"+output_file)
