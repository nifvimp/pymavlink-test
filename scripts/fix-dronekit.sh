#!/usr/bin/env bash

COLLECTIONS_ABC_ITEMS=("Container" "Hashable" "Iterable" "Iterator" "Reversible" "Generator" "Sized"   \
                       "Callable" "Collection" "Sequence" "MutableSequence" "ByteString" "Set"         \
                       "MutableSet" "Mapping" "MutableMapping" "MappingView" "ItemsView" "KeysView"    \
                       "ValuesView" "Awaitable" "Coroutine" "AsyncIterable" "AsyncIterator"            \
                       "AsyncGenerator" "Buffer")
COLLECTIONS_ABC_REGEX="\\b\\($(IFS='|'; join="${COLLECTIONS_ABC_ITEMS[*]}"; echo "${join//|/\\|}")\\)\\b"
PYTHON_BINARY="${1:-$(command -v python3 || command -v python)}" || { echo "python not found."; exit 1; }
SITE_PACKAGES_DIR=$($PYTHON_BINARY -m pip show dronekit | sed -n 's/^Location: //p')

# DeprecationWarning: Using or importing the ABCs from 'collections' instead of from 'collections.abc'
#                     is deprecated since Python 3.3, and in 3.10 it will stop working
if [[ $($PYTHON_BINARY -V 2>&1) =~ ^Python\ 3\.([1-9]+[0-9]+)\.[0-9]+$ ]]; then
	find "$SITE_PACKAGES_DIR"/dronekit -type f -name "*.py" -exec                                        \
		sed -i -e "s/^from collections import \($COLLECTIONS_ABC_REGEX\)/from collections.abc import \1/"  \
		-e "s/collections\.\($COLLECTIONS_ABC_REGEX\)/collections.abc\.\1/"                                \
		{} +
fi
