/* Copyright 2013 Bernhard R. Fischer, 2048R/5C5FFD47 <bf@abenteuerland.at>
 *
 * This file is part of Parsefsh.
 *
 * Parsefsh is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3 of the License.
 *
 * Parsefsh is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Parsefsh. If not, see <http://www.gnu.org/licenses/>.
 */

/*! This file contains support for the Raymarine FSH format.
 *
 *  @author Bernhard R. Fischer
 *          Scott Emmons - gpsbabel integration
 */

// TODO: Header block conformity for gpsbabel

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <time.h>

#include "defs.h"
#include "raymarine_fsh.h"

#define MYNAME "FSH"

static gbfile* fin;

static
arglist_t raymarine_fsh_args[] = {
  ARG_TERMINATOR
};

#ifdef HAVE_VLOG
void vlog(const char*, ...) __attribute__((format (printf, 1, 2)));
#else
#define vlog(x...) fprintf(stderr, ## x)
#endif

#define TBUFLEN 24
#define DEGSCALE (M_PI / 180.0)
#define DEG2RAD(x) ((x) * DEGSCALE)
#define RAD2DEG(x) ((x) / DEGSCALE)
#define DEG2M(x) ((x) * 60 * 1852)
#define CELSIUS(x) ((double) (x) / 100.0 - 273.15)

/*************************************************
* %%%        Internal functions              %%% *
**************************************************/

char *guid_to_string(uint64_t guid)
{
   static char buf[32];

   snprintf(buf, sizeof(buf),  "%"PRIu64"-%"PRIu64"-%"PRIu64"-%"PRIu64,
         guid >> 48, (guid >> 32) & 0xffff, (guid >> 16) & 0xffff, guid & 0xffff);
   return buf;
}


/*! This function converts an FSH timestamp into string representation.
 * @param ts Pointer to FSH timestamp.
 * @param buf Pointer to buffer which will receive the string.
 * @paran len Length of buffer.
 * @return Returns the number of bytes placed into buf without the trailing \0.
 */
int fsh_timetostr(const fsh_timestamp_t *ts, char *buf, int len)
{
   time_t t = (time_t) ts->date * 3600 * 24 + ts->timeofday;
   struct tm *tm = gmtime(&t);
   return strftime(buf, len, "%Y-%m-%dT%H:%M:%SZ", tm);
}


/*! Read add parse file header.
 *  @param fd Open file descriptor.
 *  @param fhdr Pointer to fsh_file_header_t which will be filled by this
 *  function.
 *  @return Returns 0 if it es a file header (the first one). The function returns
 *  -1 in case of error. If an I/O error occurs the function does not return.
 */
int fsh_read_file_header(gbfile* fin, fsh_file_header_t *fhdr)
{
   int len;

   if ((len = gbfread(fhdr, 1, sizeof(*fhdr), fin)) < 1)
      perror("read"), exit(EXIT_FAILURE);

   if (len < (int) sizeof(*fhdr))
      fprintf(stderr, "# file header truncated, read %d of %d\n", len, (int) sizeof(*fhdr)),
         exit(EXIT_FAILURE);

   if (memcmp(fhdr->rl90, RL90_STR, strlen(RL90_STR)))
      return -1;

   return 0;
}


/*! This reads a flob header. It works like fsh_read_file_header(). */
int fsh_read_flob_header(gbfile* fin, fsh_flob_header_t *flobhdr)
{
   int len;

   if ((len = gbfread(flobhdr, 1, sizeof(*flobhdr), fin)) < 1)
      perror("read"), exit(EXIT_FAILURE);

   if (len < (int) sizeof(*flobhdr))
      fprintf(stderr, "# flob header truncated, read %d of %d\n", len, (int) sizeof(*flobhdr)),
         exit(EXIT_FAILURE);

   if (memcmp(flobhdr->rflob, RFLOB_STR, strlen(RFLOB_STR)))
      return -1;

   return 0;
}


static int fsh_block_count(const fsh_block_t *blk)
{
   int blk_cnt;

   if (blk == NULL)
      return 0;

   for (blk_cnt = 0; blk->hdr.type != FSH_BLK_ILL; blk++, blk_cnt++);

   return blk_cnt;
}


/*! This function reads all blocks into a fsh_block_t list. The type of the
 * last block (which does not contain data anymore) is set to 0xffff.
 * @return Returns a pointer to the first fsh_block_t. The list MUST be freed
 * by the caller again with a call to free and the pointer to the first block.
 */
fsh_block_t *fsh_block_read(gbfile* fin, fsh_block_t *blk)
{
   int blk_cnt, len, pos, rlen;
   //fsh_block_t *blk = NULL;
   off_t off;

   if ((off = gbfseek(fin, 0, SEEK_CUR)) == -1)
      perror("lseek"), exit(EXIT_FAILURE);

   blk_cnt = fsh_block_count(blk);

   for (pos = 0; ; blk_cnt++)   // 0x2a is the start offset after the file header
   {
      if ((blk = (fsh_block_t*) realloc(blk, sizeof(*blk) * (blk_cnt + 1))) == NULL)
         perror("realloc"), exit(EXIT_FAILURE);
      blk[blk_cnt].data = NULL;

      // check if there's enough space left in the FLOB
      if (pos + sizeof(fsh_block_header_t) + sizeof(fsh_flob_header_t) > FLOB_SIZE)
      {
         blk[blk_cnt].hdr.type = FSH_BLK_ILL;
         vlog("end, FLOB full\n");
         break;
      }

      if ((len = gbfread(&blk[blk_cnt].hdr, 1, sizeof(blk[blk_cnt].hdr), fin)) < 1)
         perror("read"), exit(EXIT_FAILURE);

      vlog("offset = $%08lx, pos = $%04x, block type = 0x%02x, len = %d, guid %s\n",
            pos + (long) off, pos, blk[blk_cnt].hdr.type, blk[blk_cnt].hdr.len, guid_to_string(blk[blk_cnt].hdr.guid));
      pos += len;

      if (len < (int) sizeof(blk[blk_cnt].hdr))
      {
         vlog("header truncated, read %d of %d\n", len, (int) sizeof(blk[blk_cnt].hdr));
         blk[blk_cnt].hdr.type = FSH_BLK_ILL;
      }

      if (blk[blk_cnt].hdr.type == FSH_BLK_ILL)
      {
            vlog("end, empty block\n");
            break;
      }

      rlen = blk[blk_cnt].hdr.len + (blk[blk_cnt].hdr.len & 1);  // pad odd blocks by 1 byte
      if ((blk[blk_cnt].data = malloc(rlen)) == NULL)
         perror("malloc"), exit(EXIT_FAILURE);

      if ((len = gbfread(blk[blk_cnt].data, 1, rlen, fin)) < 1)
         perror("read"), exit(EXIT_FAILURE);
      pos += len;

      if (len < rlen)
      {
         vlog("block data truncated, read %d of %d\n", len, rlen);
         // clear unfilled partition of block
         memset((fsh_block_t*) blk[blk_cnt].data + len, 0, rlen - len);
         break;
      }
   }
   return blk;
}


// FIXME: if GUID cross pointers in FSH file are incorrect, program will not
// work correctly.
static void fsh_tseg_decode0(const fsh_block_t *blk, track_t *trk)
{
   int i;

   vlog("decoding tracks\n");
   for (; blk->hdr.type != FSH_BLK_ILL; blk++)
      if (blk->hdr.type == FSH_BLK_TRK)
         for (i = 0; i < trk->mta->guid_cnt; i++)
            if (blk->hdr.guid == trk->mta->guid[i])
            {
               trk->tseg[i].bhdr = (fsh_block_header_t*) &blk->hdr;
               trk->tseg[i].hdr = (fsh_track_header_t*) blk->data;
               trk->tseg[i].pt = (fsh_track_point_t*) (trk->tseg[i].hdr + 1);
            }
}


static void fsh_tseg_decode(const fsh_block_t *blk, track_t *trk, int trk_cnt)
{
   for (; trk_cnt; trk_cnt--, trk++)
      fsh_tseg_decode0(blk, trk);
}


/*! This function decodes track blocks (0x0d and 0x0e) into a track_t structure.
 * @param blk Pointer to the first fsh block.
 * @param trk Pointer to a track_t pointer. This variable will receive a
 * pointer to the first track. The caller must free the track list again with a
 * pointer to the first track.
 * @return Returns the number of tracks that have been decoded.
 */
static int fsh_track_decode0(const fsh_block_t *blk, track_t **trk)
{
   int trk_cnt = 0;

   vlog("decoding track metas\n");
   for (*trk = NULL; blk->hdr.type != FSH_BLK_ILL; blk++)
   {
      vlog("decoding 0x%02x\n", blk->hdr.type);
      if (blk->hdr.type == FSH_BLK_MTA)
      {
         vlog("track meta\n");

         if ((*trk = (track_t*) realloc(*trk, sizeof(**trk) * (trk_cnt + 1))) == NULL)
            perror("realloc"), exit(EXIT_FAILURE);

         (*trk)[trk_cnt].bhdr = (fsh_block_header_t*) &blk->hdr;
         (*trk)[trk_cnt].mta = (fsh_track_meta_t*) blk->data;

         if (((*trk)[trk_cnt].tseg = (track_segment_t*) malloc(sizeof(*(*trk)[trk_cnt].tseg) * (*trk)[trk_cnt].mta->guid_cnt)) == NULL)
            perror("malloc"), exit(EXIT_FAILURE);

         trk_cnt++;
      }
   }

   return trk_cnt;
}


int fsh_track_decode(const fsh_block_t *blk, track_t **trk)
{
   int trk_cnt;

   trk_cnt = fsh_track_decode0(blk, trk);
   fsh_tseg_decode(blk, *trk, trk_cnt);

   return trk_cnt;
}


/*! This function decodes route blocks (0x21) into a route21_t structure.
 * @param blk Pointer to the first fsh block.
 * @param trk Pointer to a route21_t pointer. This variable will receive a
 * pointer to the first route. The caller must free the route list again with a
 * pointer to the first route.
 * @return Returns the number of routes that have been decoded.
 */
int fsh_route_decode(const fsh_block_t *blk, route21_t **rte)
{
   int rte_cnt = 0;

   vlog("decoding routes\n");
   for (*rte = NULL; blk->hdr.type != FSH_BLK_ILL; blk++)
   {
      vlog("decoding 0x%02x\n", blk->hdr.type);
      switch (blk->hdr.type)
      {
         case FSH_BLK_RTE:
            vlog("route21\n");
            if ((*rte = (route21_t*) realloc(*rte, sizeof(**rte) * (rte_cnt + 1))) == NULL)
               perror("realloc"), exit(EXIT_FAILURE);

            (*rte)[rte_cnt].bhdr = (fsh_block_header_t*) &blk->hdr;
            (*rte)[rte_cnt].hdr = (fsh_route21_header_t*) blk->data;
            (*rte)[rte_cnt].guid = (int64_t*) ((char*) ((*rte)[rte_cnt].hdr + 1) + (*rte)[rte_cnt].hdr->name_len + (*rte)[rte_cnt].hdr->cmt_len);
            (*rte)[rte_cnt].hdr2 = (struct fsh_hdr2*) ((*rte)[rte_cnt].guid + (*rte)[rte_cnt].hdr->guid_cnt);
            (*rte)[rte_cnt].pt = (struct fsh_pt*) ((*rte)[rte_cnt].hdr2 + 1);
            (*rte)[rte_cnt].hdr3 = (struct fsh_hdr3*) ((*rte)[rte_cnt].pt + (*rte)[rte_cnt].hdr->guid_cnt);
            (*rte)[rte_cnt].wpt = (fsh_route_wpt_t*) ((*rte)[rte_cnt].hdr3 + 1);

            rte_cnt++;
            break;
      }
   }
   return rte_cnt;
}


/*! Free all data pointers within the block list. This MUST be called before
 * the block list is freed itself.
 * @param blk Pointer to the first block.
 */
void fsh_free_block_data(fsh_block_t *blk)
{
   for (; blk->hdr.type != FSH_BLK_ILL; blk++)
      free(blk->data);
}



/*! This function derives the ellipsoid parameters from the semi-major and
 * semi-minor axis.
 * @param el A pointer to an ellipsoid structure. el->a and el->b MUST be
 * pre-initialized.
 */
void init_ellipsoid(ellipsoid_t *el)
{
   el->e = sqrt(1 - pow(el->b / el->a, 2));
}


/*! This function calculates the nearest geographic latitude to the reference
 * latitude phi0 from the Northing N based on the ellipsoid el. It must be
 * called iteratively to gain an appropriate accuracy.
 * @param el Pointer to the ellipsoid data.
 * @param N Northing according to the Mercator projection.
 * @param phi0 Reference latitude in radians. If phi0 = 0 (e.g. initially) a
 * spherical reverse Mercator is calculated.
 * @return Returns the latitude in radians.
 */
static double phi_rev_merc(const ellipsoid_t *el, double N, double phi0)
{
   double esin = el->e * sin(phi0);
   return M_PI_2 - 2.0 * atan(exp(-N / el->a) * pow((1 - esin) / (1 + esin), el->e / 2));
}


/*! This function derives the geographic latitude from the Mercator Northing N.
 * It iteratively calls phi_rev_merc(). At a maximum it iterates either MAX_IT
 * times or until the accuracy is less than IT_ACCURACY.
 * @param el Pointer to the ellipsoid data.
 * @param N Mercator Northing.
 * @return Returns the latitude in radians.
 */
double phi_iterate_merc(const ellipsoid_t *el, double N)
{
   double phi, phi0;
   int i;

   for (phi = 0, phi0 = M_PI, i = 0; fabs(phi - phi0) > IT_ACCURACY && i < MAX_IT; i++)
   {
      phi0 = phi;
      phi = phi_rev_merc(el, N, phi0);
   }

   return phi;
}


double northing(const ellipsoid_t *el, double lat)
{
   return el->a * log(tan(M_PI_4 + lat / 2) * pow((1 - el->e * sin(lat)) / (1 + el->e * sin(lat)), el->e / 2));
}


/*! Calculate bearing and distance from src to dst.
 *  @param src Source coodinates (struct coord).
 *  @param dst Destination coordinates (struct coord).
 *  @return Returns a struct pcoord. Pcoord contains the orthodrome distance in
 *  degrees and the bearing, 0 degress north, clockwise.
 */
struct pcoord coord_diff(const struct coord *src, const struct coord *dst)
{
   struct pcoord pc;
   double dlat, dlon;

   dlat = dst->lat - src->lat;
   dlon = (dst->lon - src->lon) * cos(DEG2RAD((src->lat + dst->lat) / 2.0));

   pc.bearing = RAD2DEG(atan2(dlon, dlat));
   pc.dist = RAD2DEG(acos(
      sin(DEG2RAD(src->lat)) * sin(DEG2RAD(dst->lat)) +
      cos(DEG2RAD(src->lat)) * cos(DEG2RAD(dst->lat)) * cos(DEG2RAD(dst->lon - src->lon))));

   if (pc.bearing  < 0)
      pc.bearing += 360.0;

   return pc;
}
/*! Latitude and longitude in 0x0e track blocks are given in Mercator Northing
 * and Easting but are prescaled by some factors. This function reverses the
 * scaling.
 * @param lat0 Scaled Northing from 0x0e track point.
 * @param lon0 Scaled Easting from 0x0e track point.
 * @param lat Variable to receive result Northing.
 * @param lon Variable to receuve result Easting.
 */
static void raycoord_norm(int32_t lat0, int32_t lon0, double *lat, double *lon)
{
   *lat = lat0 / FSH_LAT_SCALE;
   *lon = lon0 / FSH_LON_SCALE * 180.0;
}

Waypoint* make_wpt(const fsh_wpt_data_t *wpd, const ellipsoid_t *el, int64_t guid)
{
   char tbuf[TBUFLEN];
   struct coord cd;

   raycoord_norm(wpd->north, wpd->east, &cd.lat, &cd.lon);
   cd.lat = phi_iterate_merc(el, cd.lat) * 180 / M_PI;
   fsh_timetostr(&wpd->ts, tbuf, sizeof(tbuf));

   printf("%s, %.7f, %.7f, %d, ",
         guid_to_string(guid), cd.lat, cd.lon, wpd->sym);

   if (wpd->tempr == TEMPR_NA)
      printf("N/A, ");
   else
      printf( "%.1f, ", CELSIUS(wpd->tempr));

   if (wpd->depth == DEPTH_NA)
      printf("N/A, ");
   else
      printf("%d, ", wpd->depth);

   printf("%.*s, %.*s, %s\n",
         wpd->name_len, wpd->txt_data, wpd->cmt_len, wpd->txt_data + wpd->name_len, tbuf);

  Waypoint* wpt;

  wpt = new Waypoint;
  wpt->shortname = QString().sprintf("%.*s", wpd->name_len, wpd->txt_data);
  wpt->description = QString().sprintf("%.*s", wpd->cmt_len, wpd->txt_data + wpd->name_len);
  wpt->latitude = cd.lat;
  wpt->longitude = cd.lon;
  wpt->SetCreationTime((time_t) wpd->ts.date * 3600 * 24 + wpd->ts.timeofday);
  if (wpd->tempr != TEMPR_NA)
        WAYPT_SET(wpt, temperature, CELSIUS(wpd->tempr));
  // TODO: depth units right?
  if (wpd->depth != DEPTH_NA)
        WAYPT_SET(wpt, depth, wpd->depth/100);

  return(wpt);
}

int wpt_01_output(const fsh_block_t *blk, const ellipsoid_t *el)
{
   fsh_wpt01_t *wpt;

   printf("# ----- BEGIN WAYPOINTS TYPE 0x01 -----\n"
                "# GUID, LAT, LON, SYM, TEMPR [C], DEPTH [cm], NAME, COMMENT, TIMESTAMP\n");
   for (; blk->hdr.type != 0xffff; blk++)
   {
      if (blk->hdr.type != 0x01)
         continue;

      wpt = (fsh_wpt01_t*) blk->data;
      waypt_add(make_wpt(&wpt->wpd, el, wpt->guid));
   }
   printf("# ----- END WAYPOINTS TYPE 0x01 -----\n");
   return 0;
}

int route_output(const route21_t *rte, int cnt, const ellipsoid_t *el)
{
   fsh_route_wpt_t *wpt;
   route_head *route;
   int i, j;

   for (j = 0; j < cnt; j++)
   {
      printf("# route '%.*s', guid_cnt = %d\n", rte[j].hdr->name_len, NAME(*rte[j].hdr), rte[j].hdr->guid_cnt);
      for (i = 0; i < rte[j].hdr->guid_cnt; i++)
         printf("#   %s\n", guid_to_string(rte[j].guid[i]));

      printf("# lat0 = %.7f, lon0 = %.7f, lat1 = %.7f, lon1 = %.7f\n# hdr2: ",
            (double) rte[j].hdr2->lat0 / 1E7, (double) rte[j].hdr2->lon0 / 1E7,
            (double) rte[j].hdr2->lat1 / 1E7, (double) rte[j].hdr2->lon1 / 1E7);
      //hexdump((char*) rte[j].hdr2 + 16, sizeof(*rte[j].hdr2) - 16);
      printf("# hdr2 [dec]: %d, %d\n", rte[j].hdr2->a, rte[j].hdr2->c);

      for (i = 0; i < rte[j].hdr->guid_cnt; i++)
         printf("# %d, %d, %d, %d, %d\n", rte[j].pt[i].a, rte[j].pt[i].b, rte[j].pt[i].c, rte[j].pt[i].d, rte[j].pt[i].sym);

      printf("# wpt_cnt %d\n", rte[j].hdr3->wpt_cnt);
      printf("# guid_cnt %d\n", rte[j].hdr->guid_cnt);

      route = route_head_alloc();
      route_add_head(route);
      route->rte_name = QString().sprintf("%.*s", rte[j].hdr->name_len, NAME(*rte[j].hdr));

      for (i = 0, wpt = rte[j].wpt; i < rte[j].hdr3->wpt_cnt; i++)
      {
         route_add_wpt(route, make_wpt(&wpt->wpt.wpd, el, wpt->guid));
         wpt = (fsh_route_wpt_t*) ((char*) wpt + wpt->wpt.wpd.name_len + wpt->wpt.wpd.cmt_len + sizeof(*wpt));
      }

   }
   return 0;
}
int track_output(const track_t *trk, int cnt, const ellipsoid_t *el)
{
   struct coord cd, cd0;
   struct pcoord pc = {0, 0};
   double dist, dist_seg;
   int i, j, k, n;
   route_head *track;

   for (j = 0; j < cnt; j++)
   {
      printf("# ----- BEGIN TRACK -----\n");
      if (trk[j].mta != NULL)
      {
         printf("# name = '%.*s', tempr_start = %.1f, depth_start = %d, tempr_end = %.1f, depth_end = %d, length = %d m, guid_cnt = %d\n",
               (int) sizeof(trk[j].mta->name), trk[j].mta->name != NULL ? trk[j].mta->name : "",
               CELSIUS(trk[j].mta->tempr_start), trk[j].mta->depth_start,
               CELSIUS(trk[j].mta->tempr_end), trk[j].mta->depth_end,
               trk[j].mta->length, trk[j].mta->guid_cnt);
         for (i = 0; i < trk[j].mta->guid_cnt; i++)
            printf("# guid[%d] = %s\n", i, guid_to_string(trk[j].mta->guid[i]));
      }
      else
         printf("# no track meta data\n");

      printf("# CNT, NR, FSH-N, FSH-E, lat, lon, DEPTH [cm], TEMPR [C], C, bearing, distance [m], TRACKNAME\n");

      track = route_head_alloc();
      track_add_head(track);
      track->rte_name = QString().sprintf("%.*s", (int) sizeof(trk[j].mta->name), trk[j].mta->name != NULL ? trk[j].mta->name : "");

      for (k = 0, n = 0, dist = 0; k < trk[j].mta->guid_cnt; k++, dist += dist_seg)
      {
         printf("# ----- BEGIN TRACKSEG -----\n");
         for (i = 0, dist_seg = 0; i < trk[j].tseg[k].hdr->cnt; i++, n++)
         {
            if (trk[j].tseg[k].pt[i].c == -1)
               continue;

            cd0 = cd;
            raycoord_norm(trk[j].tseg[k].pt[i].north, trk[j].tseg[k].pt[i].east, &cd.lat, &cd.lon);
            cd.lat = phi_iterate_merc(el, cd.lat) * 180 / M_PI;

            if (i)
               pc = coord_diff(&cd0, &cd);

            printf("%d, %d, %d, %d, %.8f, %.8f, %d, %.1f, %d, %.1f, %.1f",
                  n, i, trk[j].tseg[k].pt[i].north, trk[j].tseg[k].pt[i].east,
                  cd.lat, cd.lon, trk[j].tseg[k].pt[i].depth, CELSIUS(trk[j].tseg[k].pt[i].tempr),
                  trk[j].tseg[k].pt[i].c, pc.bearing, DEG2M(pc.dist));
            if (trk[j].mta)
               printf(", %.*s\n",
                  (int) sizeof(trk[j].mta->name), trk[j].mta->name != NULL ? trk[j].mta->name : "");
            else
               printf("\n");
            dist_seg += pc.dist;

            Waypoint* wpt;

            wpt = new Waypoint;
            //wpt->shortname = QString().sprintf("%.*s", wpd->name_len, wpd->txt_data);
            //wpt->description = QString().sprintf("%.*s", wpd->cmt_len, wpd->txt_data + wpd->name_len);
            wpt->latitude = cd.lat;
            wpt->longitude = cd.lon;
	    // TODO: No timestamp on trackpoints?
            //wpt->SetCreationTime((time_t) wpd->ts.date * 3600 * 24 + wpd->ts.timeofday);
            if (trk[j].tseg[k].pt[i].tempr != TEMPR_NA)
                WAYPT_SET(wpt, temperature, CELSIUS(trk[j].tseg[k].pt[i].tempr));
            // TODO: depth units right?
            if (trk[j].tseg[k].pt[i].depth != DEPTH_NA)
                WAYPT_SET(wpt, depth, trk[j].tseg[k].pt[i].depth/100);
            track_add_wpt(track, wpt);
         }
         printf("# distance = %.1f nm, %.1f m\n", dist_seg * 60, DEG2M(dist_seg));
         printf("# ----- END TRACKSEG -----\n");
      }
      printf("# total distance = %.1f nm, %.1f m\n", dist * 60, DEG2M(dist));
      printf("# ----- END TRACK -----\n");
   }
   return 0;
}


/*******************************************************************************
* %%%        global callbacks called by gpsbabel main process              %%% *
*******************************************************************************/

static void
raymarine_fsh_rd_init(const QString& fname)
{
  fin = gbfopen(fname, "rb", MYNAME);
}

static void
raymarine_fsh_rd_deinit(void)
{
  gbfclose(fin);
}

static void
raymarine_fsh_read(void)
{
  fsh_file_header_t fhdr;
  fsh_flob_header_t flobhdr;
  fsh_block_t *blk = NULL;
  route21_t *rte;
  track_t *trk;
  ellipsoid_t el = WGS84;
  int flob_cnt = 0, rte_cnt = 0, trk_cnt = 0;

  init_ellipsoid(&el);

  if (fsh_read_file_header(fin, &fhdr) == -1)
     fatal(MYNAME ": No RL90 header\n");
  vlog("filer header values 0x%04x\n", fhdr.flobs);

  vlog("reading flob %d\n", flob_cnt);
  while (fsh_read_flob_header(fin, &flobhdr) != -1)
  {
     blk = fsh_block_read(fin, blk);

     // try to read next FLOB
     flob_cnt++;
     if (flob_cnt >= fhdr.flobs)
        break;
     if (gbfseek(fin, flob_cnt * FLOB_SIZE + sizeof(fhdr), SEEK_SET) == -1)
        fatal(MYNAME ": File seek error\n");
  }

  rte_cnt = fsh_route_decode(blk, &rte);
  trk_cnt = fsh_track_decode(blk, &trk);

  wpt_01_output((fsh_block_t*) blk, &el);

  track_output(trk, trk_cnt, &el);
  route_output(rte, rte_cnt, &el);

  fsh_free_block_data(blk);
  free(blk);

// Tracks are just like routes, except the word "track" replaces "routes".
//
}

/**************************************************************************/

ff_vecs_t raymarine_fsh_vecs = {
  ff_type_file,
  {
    ff_cap_read 	/* waypoints */,
    ff_cap_read 	/* tracks */,
    ff_cap_read 	/* routes */
  },
  raymarine_fsh_rd_init,
  NULL,
  raymarine_fsh_rd_deinit,
  NULL,
  raymarine_fsh_read,
  NULL,
  NULL,
  raymarine_fsh_args,
  CET_CHARSET_ASCII, 1
};
/**************************************************************************/
