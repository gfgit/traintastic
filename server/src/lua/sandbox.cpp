/**
 * server/src/lua/sandbox.cpp - Lua sandbox
 *
 * This file is part of the traintastic source code.
 *
 * Copyright (C) 2019-2020 Reinder Feenstra
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "sandbox.hpp"
#include "push.hpp"
#include "object.hpp"
#include "method.hpp"
#include "log.hpp"
#include "class.hpp"
#include "to.hpp"
#include <version.hpp>
#include <traintastic/utils/str.hpp>
#include <traintastic/codename.hpp>
#include "../world/world.hpp"
#include "../enum/decoderprotocol.hpp"
#include "../enum/direction.hpp"
#include "../enum/worldevent.hpp"
#include "../enum/worldscale.hpp"
#include "../set/worldstate.hpp"

#define LUA_SANDBOX "_sandbox"
#define LUA_SANDBOX_GLOBALS "_sandbox_globals"

#define ADD_GLOBAL_TO_SANDBOX(x) \
  lua_pushliteral(L, x); \
  lua_getglobal(L, x); \
  lua_settable(L, -3);

constexpr std::array<std::string_view, 17> readOnlyGlobals = {{
  // Lua baselib:
  "assert",
  "type",
  "pairs",
  "ipairs",
  "_G",
  // Constants:
  "VERSION",
  "VERSION_MAJOR",
  "VERSION_MINOR",
  "VERSION_PATCH",
  "CODENAME",
  "LUA_VERSION",
  // Objects:
  "world",
  "log",
  // Functions:
  "is_instance",
  // Type info:
  "class",
  "enum",
  "set",
}};

namespace Lua {

void Sandbox::close(lua_State* L)
{
  delete *static_cast<StateData**>(lua_getextraspace(L)); // free state data
  lua_close(L);
}

int Sandbox::__index(lua_State* L)
{
  lua_getglobal(L, LUA_SANDBOX_GLOBALS);
  lua_replace(L, 1);

  lua_rawget(L, 1);

  return 1;
}

int Sandbox::__newindex(lua_State* L)
{
  if(std::find(readOnlyGlobals.begin(), readOnlyGlobals.end(), to<std::string_view>(L, 2)) != readOnlyGlobals.end())
    errorGlobalNIsReadOnly(L, lua_tostring(L, 2));

  lua_getglobal(L, LUA_SANDBOX_GLOBALS);
  lua_replace(L, 1);

  lua_rawset(L, 1);

  return 0;
}

SandboxPtr Sandbox::create(Script& script)
{
  lua_State* L = luaL_newstate();

  // load Lua baselib:
  lua_pushcfunction(L, luaopen_base);
  lua_pushliteral(L, "");
  lua_call(L, 1, 0);

  // create state data:
  *static_cast<StateData**>(lua_getextraspace(L)) = new StateData(script);

  // register types:
  Enum<DecoderProtocol>::registerType(L);
  Enum<Direction>::registerType(L);
  Enum<WorldEvent>::registerType(L);
  Enum<WorldScale>::registerType(L);
  Set<WorldState>::registerType(L);
  Object::registerType(L);
  Method::registerType(L);

  // setup sandbox:
  lua_newtable(L);
  luaL_newmetatable(L, LUA_SANDBOX);
  lua_pushcfunction(L, __index);
  lua_setfield(L, -2, "__index");
  lua_pushcfunction(L, __newindex);
  lua_setfield(L, -2, "__newindex");
  lua_setmetatable(L, -2);
  lua_setglobal(L, LUA_SANDBOX);

  // setup globals:
  lua_newtable(L);

  // add some Lua baselib functions to the sandbox:
  ADD_GLOBAL_TO_SANDBOX("assert")
  ADD_GLOBAL_TO_SANDBOX("type")
  ADD_GLOBAL_TO_SANDBOX("pairs")
  ADD_GLOBAL_TO_SANDBOX("ipairs")

  // set VERSION:
  lua_pushstring(L, TRAINTASTIC_VERSION);
  lua_setfield(L, -2, "VERSION");
  lua_pushinteger(L, TRAINTASTIC_VERSION_MAJOR);
  lua_setfield(L, -2, "VERSION_MAJOR");
  lua_pushinteger(L, TRAINTASTIC_VERSION_MINOR);
  lua_setfield(L, -2, "VERSION_MINOR");
  lua_pushinteger(L, TRAINTASTIC_VERSION_PATCH);
  lua_setfield(L, -2, "VERSION_PATCH");

  // set CODENAME
  lua_pushstring(L, TRAINTASTIC_CODENAME);
  lua_setfield(L, -2, "CODENAME");

  // set LUA_VERSION
  const std::string_view ident{lua_ident};
  push(L, ident.substr(13, ident.find('$', 13) - 14));
  lua_setfield(L, -2, "LUA_VERSION");

  // add world:
  push(L, std::static_pointer_cast<::Object>(script.world().lock()));
  lua_setfield(L, -2, "world");

  // add logger:
  Log::push(L);
  lua_setfield(L, -2, "log");

  // add is_instance function:
  lua_pushcfunction(L, Class::isInstance);
  lua_setfield(L, -2, "is_instance");

  // add class types:
  lua_newtable(L);
  Class::registerValues(L);
  ReadOnlyTable::wrap(L, -1);
  lua_setfield(L, -2, "class");

  // add enum values:
  lua_newtable(L);
  Enum<DecoderProtocol>::registerValues(L);
  Enum<Direction>::registerValues(L);
  Enum<WorldEvent>::registerValues(L);
  Enum<WorldScale>::registerValues(L);
  ReadOnlyTable::wrap(L, -1);
  lua_setfield(L, -2, "enum");

  // add set values:
  lua_newtable(L);
  Set<WorldState>::registerValues(L);
  ReadOnlyTable::wrap(L, -1);
  lua_setfield(L, -2, "set");

  // let global _G point to the sandbox:
  lua_getglobal(L, LUA_SANDBOX);
  lua_setfield(L, -2, "_G");

  lua_setglobal(L, LUA_SANDBOX_GLOBALS);

  return SandboxPtr(L, close);
}

Sandbox::StateData& Sandbox::getStateData(lua_State* L)
{
  return **static_cast<StateData**>(lua_getextraspace(L));
}

int Sandbox::getGlobal(lua_State* L, const char* name)
{
  lua_getglobal(L, LUA_SANDBOX); // get the sandbox
  lua_pushstring(L, name);
  const int type = lua_gettable(L, -2); // get item
  lua_insert(L, lua_gettop(L) - 1); // swap item and sandbox on the stack
  lua_pop(L, 1); // remove sandbox from the stack
  return type;
}

int Sandbox::pcall(lua_State* L, int nargs, int nresults, int errfunc)
{
  // check if the function has _ENV as first upvalue
  // if so, replace it by the sandbox
  // NOTE: functions which don't use any globals, don't have an _ENV !!
  assert(lua_isfunction(L, -(1 + nargs)));
  const char* name = lua_getupvalue(L, -(1 + nargs), 1);
  if(name)
    lua_pop(L, 1); // remove upvalue from stack
  if(name && strcmp(name, "_ENV") == 0)
  {
    lua_getglobal(L, LUA_SANDBOX); // get the sandbox
    assert(lua_istable(L, -1));
    if(!lua_setupvalue(L, -(2 + nargs), 1)) // change _ENV to the sandbox
    {
      assert(false); // should never happen
      lua_pop(L, 2 + nargs); // clear stack
      lua_pushliteral(L, "Internal error @ " __FILE__ ":" STR(__LINE__));
      return LUA_ERRRUN;
    }
  }
  return lua_pcall(L, nargs, nresults, errfunc);
}

}
