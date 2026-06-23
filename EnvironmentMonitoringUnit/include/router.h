#ifndef ROUTER_H
#define ROUTER_H

#include "config.h"

/**
 * Generic any-interface -> any-interface echo router.
 *
 * Forwards bytes per the ROUTES[] table (defined in src/router.cpp): each
 * {from, to} entry copies bytes received on 'from' out to 'to'. Non-blocking
 * and bounded per tick so no single route can starve the loop.
 */

/** A directed echo route: bytes from 'from' are forwarded to 'to'. */
struct Route {
    InterfaceId from;
    InterfaceId to;
};

/** Call once from setup(). */
void router_setup(void);

/** Non-blocking tick: pump every configured route. */
void router_update(void);

#endif // ROUTER_H
