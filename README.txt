Care-Move FMS: Physical SSH-only dashboard patch

1. Extract the zip on swPC.
2. In FMS root, apply:
   python3 /path/to/extracted/fms_physical_only_update/apply_update.py /home/sw/DevSpace/FMS
3. Ensure noninteractive SSH login from swPC (no password prompt):
   ssh -o BatchMode=yes -o StrictHostKeyChecking=yes pinky@192.168.0.83 true
4. Backend terminal:
   cd /home/sw/DevSpace/FMS
   export FMS_PHYSICAL_SSH_TARGETS='pinky-1=pinky@192.168.0.83'
   export FMS_AUTO_TICK=false
   export FMS_SEED_FLEET=false
   ./scripts/run_api.sh
5. Frontend terminal:
   cd /home/sw/DevSpace/FMS
   ./scripts/run_dashboard.sh
6. GET http://localhost:8741/api/physical/robots
   Only authenticated SSH robots appear in robots[].

NOTE: FMS_AUTO_TICK=false and FMS_SEED_FLEET=false prevent new mock seeding and scheduling, but existing historical mock records remain in the database. The new frontend doesn't fetch them. No physical robot Task/ROS/pose integration is implied. No SSH passwords/private keys are stored in code.
