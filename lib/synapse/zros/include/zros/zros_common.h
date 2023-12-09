/*
 * Copyright (c) 2023 CogniPilot Foundation
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZROS_COMMON_H
#define ZROS_COMMON_H

#define ZROS_OK 0;

#define ZROS_RC(X, Y) \
    {                 \
        int rc = X;   \
        if (rc < 0) { \
            Y;        \
        }             \
    };

#endif // ZROS_COMMON_H
// vi: ts=4 sw=4 et
