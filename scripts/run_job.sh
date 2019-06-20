#!/usr/bin/env bash
#SBATCH -J emergent_behavior
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem 8G
#SBATCH --gres=gpu:0
#SBATCH -t 3:00:00
#SBATCH -C E5-2680

# Stop execution after any error
set -e

# Your user name
# (Don't change this)
MYUSER=$(whoami)

# Cleanup function to be executed upon exit, for any reason
function cleanup() {
    mkdir -p /home/${MYUSER}/ERRORS
    cp * /home/${MYUSER}/ERRORS
    rm -rf ${WORKDIR}
}



########################################
#
# Useful variables
#
########################################



# Path of the local storage on a node
# Use this to avoid sending data streams over the network
# (Don't change this)
LOCALDIR=/tmp

# ARGoS environment variables
# (Don't change this)
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$HOME/argos3bundle/lib/argos3
export PATH=$PATH:$HOME/argos3bundle/bin

# Buzz environment variables
# (Don't change this)
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$HOME/buzzbundle/lib
export PATH=$PATH:$HOME/buzzbundle/bin

# Folder where you want your data to be stored
# (Adapt this to your needs)
DATADIR=/home/${MYUSER}/Experiment_Results

# Path to the file template.argos
# (Adapt this to your needs)
TEMPLATE=experiments/emergent_behavior.argos



########################################
#
# Job-related variables
#
########################################

# Parameters related to this job
RAND_SEED=$1

# Job id
# (Change this to reflect the above parameters)
THISJOB=${RAND_SEED}

# Job working directory
# (Don't change this)
WORKDIR=${LOCALDIR}/${MYUSER}/${THISJOB}



########################################
#
# Job directory
#
########################################

# Create work dir from scratch, enter it
# (Don't change this)
rm -rf ${WORKDIR} && mkdir -p ${WORKDIR} && cd ${WORKDIR}

# Make sure you cleanup upon exit
# (Don't change this)
trap cleanup EXIT SIGINT SIGTERM



########################################
#
# Actual job logic
#
########################################

cp -r /home/${MYUSER}/Emergent-Behavior-From-Changeable-Internal-State/experiments .
cp -r /home/${MYUSER}/Emergent-Behavior-From-Changeable-Internal-State/buzz .
/home/${MYUSER}/buzzbundle/bin/bzzc buzz/emergent_behavior.bzz
/home/${MYUSER}/Emergent-Behavior-From-Changeable-Internal-State/build/embedding/mpga_emergent_behavior ${RAND_SEED}

# Transfer info back to my home directory
mkdir data_${THISJOB}
mv *.dat *.csv data_${THISJOB}/
zip data_${THISJOB}.zip data_${THISJOB}/*

cp -a data_${THISJOB}.zip ${DATADIR}
