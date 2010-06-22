/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  source file for Qt circles widget

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                    GCC V3.4

  -------------------------------------------------------------------------

  Revision History:

  2008/11/19 d.k.:   start of the implementation

****************************************************************************/

#include <QPen>
#include <QPainter>
#include <QColor>

#include "Circles.h"




Circles::Circles(int iCount_p, const QPen & pen_p,
           QWidget *parent)
    : QWidget(parent)
{
QColor color;

    m_iCount = iCount_p;

    m_aPen = new QPen[iCount_p];

    for (int nIdx=0; nIdx < m_iCount; nIdx++)
    {
        // initialize the pen
        m_aPen[nIdx] = pen_p;
        // but change its color
        color.setHsv(((m_iCount - nIdx - 1) * 359 / m_iCount), 255, 255);
        m_aPen[nIdx].setColor(color);
    }

    m_uiValue = 0xA5A5;

    setBackgroundRole(QPalette::Base);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

}

void Circles::setValue(unsigned int uiDataIn_p)
{
    m_uiValue = uiDataIn_p;
    update();

/*
    int nIdx;

    for (nIdx=0; nIdx < m_iCount; nIdx++)
    {
        if (uiDataIn_p & (1 << nIdx))
        {
            m_ppLedButtons[nIdx]->setDisabled(false);
        }
        else
        {
            m_ppLedButtons[nIdx]->setDisabled(true);
        }
    }
*/
}



void Circles::paintEvent(QPaintEvent *)
{
int iDiameter;
int iWidth = width();

    QPainter painter(this);

    // disable antialiasing
    painter.setRenderHint(QPainter::Antialiasing, false);

    iWidth /= 2;

    // move local coordinate system to center of widget
    painter.translate(iWidth, height() /*/ 2*/);

    // paint circles
    for (int nIdx=0; nIdx < m_iCount; nIdx++)
    {
        if (m_uiValue & (1 << nIdx))
        {
            painter.setPen(m_aPen[nIdx]);
            iDiameter = nIdx * (iWidth / m_iCount) + iWidth;
            painter.drawArc(QRect(-iDiameter / 2, -iDiameter / 2,
                                      iDiameter, iDiameter), 0, 16 * 180);
        }
    }
}


