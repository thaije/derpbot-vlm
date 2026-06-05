#!/usr/bin/env bash
# Run run_diag.sh over a list of seeds sequentially (one sim at a time) and
# print a compact result table (target, proximity, minD, TP/FP, exploration).
#
# Usage: scripts/sweep.sh "1 2 3 4 5" [config]
set -o pipefail
SEEDS="${1:-1 2 3 4 5}"
CONFIG="${2:-config/vlm_config_cloud.yaml}"
REPO_ROOT="$HOME/Projects/derpbot-vlm"
RESULTS="$HOME/Projects/robot-sandbox/results"

for s in $SEEDS; do
    echo "######## SEED $s ########"
    before=$(ls -t "$RESULTS"/basement_find_easy_*.json 2>/dev/null | head -1)
    bash "$REPO_ROOT/scripts/run_diag.sh" "$s" "$CONFIG" 1 2>&1 | sed -n '/FUNNEL/,$p'
    # newest result for this seed
    R=$(ls -t "$RESULTS"/basement_find_easy_*.json 2>/dev/null | head -1)
    python3 -c "
import json
d=json.load(open('$R'));rm=d['raw_metrics']
gt=rm.get('ground_truth_objects',{})
tgt=[o for o in gt.values() if o.get('mission_target')]
tname=tgt[0]['type'] if tgt else '?'
succ=rm.get('proximity_success') and rm.get('target_detected')
print('RESULT seed $s | target=%s | %s | prox=%s minD=%.2f det=%d fp=%d col=%d exp%%=%.0f score=%.1f' % (
    tname, 'SUCCESS' if succ else 'no', rm['proximity_reached'], rm['min_distance_to_target'],
    int(rm['detection_count']), int(rm['false_positive_count']), int(rm['collision_count']),
    rm['exploration_coverage'], d['overall_score']))
"
done
