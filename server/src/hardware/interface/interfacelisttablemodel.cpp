/**
 * server/src/hardware/interface/interfacelisttablemodel.cpp
 *
 * This file is part of the traintastic source code.
 *
 * Copyright (C) 2021 Reinder Feenstra
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

#include "interfacelisttablemodel.hpp"
#include "interfacelist.hpp"
#include "../../utils/displayname.hpp"

constexpr uint32_t columnId = 0;
constexpr uint32_t columnName = 1;
constexpr uint32_t columnStatus = 2;

bool InterfaceListTableModel::isListedProperty(std::string_view name)
{
  return
    name == "id" ||
    name == "name" ||
    name == "status";
}

InterfaceListTableModel::InterfaceListTableModel(InterfaceList& list) :
  ObjectListTableModel<Interface>(list)
{
  setColumnHeaders({
    DisplayName::Object::id,
    DisplayName::Object::name,
    DisplayName::Interface::status,
    });
}

std::string InterfaceListTableModel::getText(uint32_t column, uint32_t row) const
{
  if(row < rowCount())
  {
    const Interface& interface = getItem(row);

    switch(column)
    {
      case columnId:
        return interface.id;

      case columnName:
        return interface.name;

      case columnStatus:
        if(const auto* it = EnumValues<InterfaceStatus>::value.find(interface.status); it != EnumValues<InterfaceStatus>::value.end())
          return std::string("$").append(EnumName<InterfaceStatus>::value).append(":").append(it->second).append("$");
        break;

      default:
        assert(false);
        break;
    }
  }

  return "";
}

void InterfaceListTableModel::propertyChanged(BaseProperty& property, uint32_t row)
{
  if(property.name() == "id")
    changed(row, columnId);
  else if(property.name() == "name")
    changed(row, columnName);
  else if(property.name() == "status")
    changed(row, columnStatus);
}
