#!/usr/bin/env bash

export API_HOST=https://api.konik.ai
export ATHENA_HOST=wss://athena.konik.ai
echo -n "2" > /data/params/d/HasAcceptedTerms
echo -n "0.2.0" > /data/params/d/CompletedTrainingVersion
echo -n "1" > /data/params/d/IsMetric
exec ./launch_chffrplus.sh
