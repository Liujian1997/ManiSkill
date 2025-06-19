#for env_id in PickCube-v1 PushCube-v1 StackCube-v1 PullCube-v1 PullCubeTool-v1 DrawTriangle-v1 PlaceSphere-v1 LiftPegUpright-v1 PegInsertionSide-v1
for env_id in PickCube-v1 PushCube-v1 StackCube-v1 PullCube-v1
do
    python  mani_skill/examples/motionplanning/panda/run.py \
        --env-id $env_id \
        --traj-name="trajectory_panda" \
        -n 5 \
        --only-count-success \
        --num-procs 8 \
        --save-video
        # --sim-backend="gpu"
done