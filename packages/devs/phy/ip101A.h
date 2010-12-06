
#ifndef _IP101A_HEADER_
#define _IP101A_HEADER_

#define MII_CTRL_REG                0x00
#define MII_STAT_REG                0x01
#define MII_PHY_ID1_REG             0x02
#define MII_PHY_ID2_REG             0x03
#define MII_AUTO_NEG_ADV_REG        0x04
#define MII_AUTO_NEG_LPA_REG        0x05
#define MII_AUTO_NEG_EXP_REG        0x06
#define MII_PHY_CTRL_REG            0x10
#define MII_PHY_IRQ_REG             0x11
#define MII_PHY_STAT_REG            0x12
#define MII_PHY_CTRL2_REG           0x1E

#define MII_PHY_IRQ_INTR	      0x8000
#define MII_PHY_IRQ_ALL_MASK		0x0800
#define MII_PHY_IRQ_SPEED_MASK      0x0400
#define MII_PHY_IRQ_DUPLEX_MASK     0x0200
#define MII_PHY_IRQ_LINK_MASK		0x0100
#define MII_PHY_IRQ_ARBITER_MASK    0x0080
#define MII_PHY_IRQ_ARBITER_CHANGE	0x0040
#define MII_PHY_IRQ_SPEED_CHANGE	0x0004
#define MII_PHY_IRQ_DUPLEX_CHANGE	0x0002
#define MII_PHY_IRQ_LINK_CHANGE     0x0001

#define IP101A_CTRL_DUPLEX          0x0100
#define IP101A_CTRL_100MB           0x2000
#define IP101A_CTRL_AUTO_NEG        0x1000
#define IP101A_CTRL_AUTO_NEG_RST    0x0200

#define IP101A_LINK_STATUS          0x0004
#define IP101A_AUTO_COMPLETED       0x20

#define IP101A_DUPLEX_MODE          0x2000
#define IP101A_SPEED_100MB          0x4000
#define IP101A_LINK_STATUS2         0x0400

#endif

