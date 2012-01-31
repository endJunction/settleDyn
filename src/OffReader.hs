{- This file is part of "settleDyn" software.

Copyright (C) 2009, 2010, 2011
          Helmholtz Centre Potsdam, GFZ German Research Centre for Geosciences.

          (C) 2011, 2012
          Dmitrij Yu. Naumov

"settleDyn" is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

"settleDyn" is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
"settleDyn".  If not, see <http://www.gnu.org/licenses/>.


Author: Dmitrij Yu. Naumov

-}


-- This provides an interface to .OFF files.  An .OFF file encodes a list of
-- points and facets. (There are other features, which are unsupported here.)

module OffReader (
    readOffFile
  ) where

import Text.ParserCombinators.Parsec
import Control.Applicative (Applicative((<*), (<*>)), (<$>))

-- OFF file parser. -----------------------------------------------------------

-- An .OFF file begins with a header contaning the number of vertices, facets
-- and edges. (Number of edges is ignored.)

parseOffFileHeader :: GenParser Char st (Int, Int)
parseOffFileHeader =
      string "OFF\n" >>                   -- skip "OFF" header
      (,) <$>                             -- construct pair of:
      number <* whitespaces <*>               -- number of vertices
      number <* anyChar `manyTill` newline    -- number of facets

-- The header is followed by data lines. At first there are points, which are
-- given as floating point numbers, one point per line.

 -- todo here is to ensure three floats
pointLine = extractLineOf float

-- Points are followed by facets, each facet given as a number of vertices in
-- the facet and the vertex numbers, one facet per line.

facetLine = checkAndExtractVertices <$> extractLineOf number
      where checkAndExtractVertices :: [Int] -> [Int]
            checkAndExtractVertices (n:vs)
                | length vs == n = vs
                | otherwise      = error "Wrong number of vertices in facet."

-- There are as many points and facets, as read in the header.

offFile =
      parseOffFileHeader >>= \(v, f) ->
      spaces >>   -- skip space btw. header and points
      (,) <$>     -- construct pair of:
      count v pointLine <*>   -- as many as v points
      count f facetLine       -- and f facets.
      <* eof

-- Given a file name, we read its content into array of points and list of facets.

readOffFile :: FilePath -> IO ([[Double]], [[Int]])
readOffFile [] = error "File name is empty."
readOffFile filename = do
      file <- readFile filename
      let ls = case parse offFile filename file of
                    Left err -> error $ "An error occured in file " ++ show filename ++
                                        ":\n" ++ show err
                    Right result -> result
      return ls

-- Parsers for numbers and lines etc. -----------------------------------------

whitespaces = many1 $ oneOf " \t"

number :: GenParser Char st Int
number = read <$> many1 digit


-- Float is difficult. There are other possibilities using Parsec.Token, but I
-- don't get 'em. For now this must be sufficient.
--
-- So float is [+|-][digits][.][digits]. There might be some strange sequences,
-- which correspond to floating point numbers.

float :: GenParser Char st Double
float = parseSign <*> parseFloat

parseFloat :: GenParser Char st Double
parseFloat =
   read <$> ((++) <$> option "0" (many1 digit) <*>
   (frac <$> option "" (char '.' >> many digit)))
    where
         frac :: String -> String
         frac [] = ".0"
         frac fs = '.' : fs

parseSign = sign <$> option '+' (oneOf "+-")
   where sign :: (Num a) => Char -> a -> a
         sign '-' = negate
         sign '+' = id

extractLineOf :: GenParser Char st a -> GenParser Char st [a]
extractLineOf what = sepEndBy1 what whitespaces <* newline
