/*
    Create .zip archives.

    Copyright (C) 2015 Robert Lipe, gpsbabel.org

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111 USA

 */

#include <QtCore/QString>
#include <QtCore/QStringList>
#include "zlib/contrib/minizip/zip.h"

class  ZipArchive
{
 public:
  ZipArchive(QString zipfile);
 ~ZipArchive();

 void Close();
 zipFile Open(QString zipfilename);
 bool Add(QString item_to_add);
 bool Add(QStringList items_to_add);

private:
  QString filename_;
  bool valid_;
  zipFile zipfile_;
};
