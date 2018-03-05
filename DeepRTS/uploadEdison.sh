#/bin/bash

fullfile=$(find src/*)
projectname=$(basename "${fullfile}" .cpp)
projectdir=$(pwd)/tmp/"${projectname}"/

mkdir -p "${projectdir}"
cp "${fullfile}" "${projectdir}""${projectname}".ino

for F in lib/*/*; do
    echo "${F}"   # your processing here
    cp "${F}" "${projectdir}"
done

pushd "${projectdir}"

arduino_debug -v --upload --board Intel:i686:izmir_ec --port COM5 "${projectdir}""${projectname}".ino

popd

rm -r tmp
