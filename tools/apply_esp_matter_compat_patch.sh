#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_DIR="$ROOT_DIR/managed_components/espressif__esp_matter/connectedhomeip/connectedhomeip/src/app"
NULLABLE_H="$BASE_DIR/data-model/Nullable.h"
OBJECTS_H="$BASE_DIR/clusters/closure-control-server/closure-control-cluster-objects.h"

if [[ ! -f "$NULLABLE_H" || ! -f "$OBJECTS_H" ]]; then
  echo "[patch] managed_components が見つかりません。先に 'idf.py reconfigure' を実行してください。" >&2
  exit 1
fi

# Patch 1: Nullable<T>::operator==(const Nullable<T>&) to avoid constrained std::optional comparison failure.
if ! grep -Fq "return Value() == other.Value();" "$NULLABLE_H"; then
  perl -0777 -i -pe 's@\Qinline bool operator==(const Nullable<T> & other) const
    {
        return static_cast<const std::optional<T> &>(*this) == static_cast<const std::optional<T> &>(other);
    }\E@inline bool operator==(const Nullable<T> & other) const
    {
        if (IsNull() != other.IsNull())
        {
            return false;
        }
        if (IsNull())
        {
            return true;
        }
        return Value() == other.Value();
    }@s' "$NULLABLE_H"
fi

# Patch 2: Add same-type operator== overloads to avoid C++20 ambiguous rewritten-candidate warnings.
if ! grep -Fq "bool operator==(const GenericOverallCurrentState & rhs) const" "$OBJECTS_H"; then
  perl -0777 -i -pe 's@\Qbool operator==(const Structs::OverallCurrentStateStruct::Type & rhs) const
    {
        return position == rhs.position && latch == rhs.latch && speed == rhs.speed && secureState == rhs.secureState;
    }\E@bool operator==(const Structs::OverallCurrentStateStruct::Type & rhs) const
    {
        return position == rhs.position && latch == rhs.latch && speed == rhs.speed && secureState == rhs.secureState;
    }

    bool operator==(const GenericOverallCurrentState & rhs) const
    {
        return position == rhs.position && latch == rhs.latch && speed == rhs.speed && secureState == rhs.secureState;
    }@s' "$OBJECTS_H"
fi

if ! grep -Fq "bool operator==(const GenericOverallTargetState & rhs) const" "$OBJECTS_H"; then
  perl -0777 -i -pe 's@\Qbool operator==(const Structs::OverallTargetStateStruct::Type & rhs) const
    {
        return position == rhs.position && latch == rhs.latch && speed == rhs.speed;
    }\E@bool operator==(const Structs::OverallTargetStateStruct::Type & rhs) const
    {
        return position == rhs.position && latch == rhs.latch && speed == rhs.speed;
    }

    bool operator==(const GenericOverallTargetState & rhs) const
    {
        return position == rhs.position && latch == rhs.latch && speed == rhs.speed;
    }@s' "$OBJECTS_H"
fi

echo "[patch] esp_matter compatibility patch applied."
